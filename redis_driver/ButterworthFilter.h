/** \file
* \copyright<br>
* Copyright (c) 2015-2016
*<br>
* Sai is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*<br>
* Sai is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*<br>
* You should have received a copy of the Lesser GNU Lesser General Public License
* along with SAI.  If not, see <http://www.gnu.org/licenses/>.
<br>
Author: Vikranth Dwaracherla <vikranth@stanford.edu>
Author: Brian Soe <bsoe@stanford.edu>
*/


#ifndef SAI_FILTER_BUTTERWORTHFILTER_H_
#define SAI_FILTER_BUTTERWORTHFILTER_H_

#include <math.h>
#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>

namespace sai {


class Filter
{
public:

    Filter(){}
    ~Filter(){}

    /** \brief Set the dimension of the input state.
     * \param dimension The dimension of the input state */
    virtual void setDimension(const int dimension) = 0;

    /** \brief Update the filter
     * \param x State vector. Each element is filtered individually. */
    virtual const Eigen::VectorXd& update(const Eigen::VectorXd& x) = 0;

};

/** \brief Butterworth filter, 2nd order
 * \ingroup filter */
class ButterworthFilter : public Filter
{
public:
    /** \brief Constructor. Call setDimension before using */
    ButterworthFilter(){}

    /** \brief Constructor.
     * \param dimension The dimension of the input state */
    ButterworthFilter(const int dimension)
    {
        dim_ = dimension;
        x0.setZero(dimension);
        x1.setZero(dimension);
        x2.setZero(dimension);
        f0.setZero(dimension);
        f1.setZero(dimension);
        f2.setZero(dimension);
    }

    /** \brief Set the dimension of the input state.
     * \param dimension The dimension of the input state */
    void setDimension(const int dimension)
    {
        dim_ = dimension;
        x0.setZero(dimension);
        x1.setZero(dimension);
        x2.setZero(dimension);
        f0.setZero(dimension);
        f1.setZero(dimension);
        f2.setZero(dimension);
    }

    /** \brief Set the cutoff frequency.
     * \param fc The normalized frequency, as a fraction of the sampling frequency. Range is [0, 0.5) */
    void setCutoffFrequency(double fc)
    {
        if (fc >= 0.5){ throw std::runtime_error("ButterworthFilter. fc should be between 0 and 0.5\n"); }
        if (fc < 0.0){ throw std::runtime_error("ButterworthFilter. fc should be between 0 and 0.5\n"); }

        fc_ = fc;
        const double ita =1.0/ tan(M_PI*fc);
        // std::cout << tan(M_PI*fc) << std::endl;
        // std:cout << "ita: " << ita << std::endl;
        const double q=sqrt(2.0);
        b0 = 1.0 / (1.0 + q*ita + ita*ita);
        b1= 2*b0;
        b2= b0;
        a0 = 1;
        a1 = -2.0 * (ita*ita - 1.0) * b0;
        a2 = (1.0 - q*ita + ita*ita) * b0;

        //std::cout << "Butterworth b: " << b0 << '\t' << b1 << '\t' << b2 << '\n';
        //std::cout << "Butterworth a: " << a0 << '\t' << a1 << '\t' << a2 << '\n';
    }

    /** \brief Update the filter
     * \param x State vector. Each element is filtered individually. */
    const Eigen::VectorXd& update(const Eigen::VectorXd& x)
    {
        x2 = x1;
        x1 = x0;
        x0 = x;
        f2 = f1;
        f1 = f0;
        for (unsigned int i=0; i<dim_; ++i){
            f0(i) = ( b0 * x0(i) + b1 * x1(i) + b2 * x2(i) - a1 * f1(i) - a2 * f2(i) ) / a0;
            // std::cout << "b0: " << b0 << std::endl;
            // std::cout << "b1: " << b1 << std::endl;
            // std::cout << "b2: " << b2 << std::endl;
            // std::cout << "a0: " << a0 << std::endl;
            // std::cout << "a1: " << a1 << std::endl;  // nan
            // std::cout << "a2: " << a2 << std::endl;  // nan
        }
        return f0;
    }

private:
    unsigned int dim_ = 0;

    Eigen::VectorXd x0; // x(t)
    Eigen::VectorXd x1; // x(t-1)
    Eigen::VectorXd x2; // x(t-2)
    Eigen::VectorXd f0; // filtered x(t)
    Eigen::VectorXd f1; // filtered x(t-1)
    Eigen::VectorXd f2; // filtered x(t-2)

    double fc_;
    double a0, a1, a2, b0, b1, b2;
};

}

#endif
