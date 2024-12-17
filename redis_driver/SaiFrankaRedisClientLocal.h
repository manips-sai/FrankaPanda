#ifndef CDATABASEREDISCLIENT_H
#define CDATABASEREDISCLIENT_H

#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <string>
#include <iostream>
#include <stdexcept>
#include <json/json.h>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace Sai {
namespace Franka {

#define CHECK(X) if ( !X || X->type == REDIS_REPLY_ERROR ) { printf("Error\n"); exit(-1); }

using std::cout;
using std::string;
using std::runtime_error;

struct HiredisServerInfo {
public:
    string hostname_;
    int port_;
    timeval timeout_;
};

class CDatabaseRedisClient {
public:
	CDatabaseRedisClient ()
		: context_(NULL),
		reply_(NULL)
	{// do nothing
	}

	// init with server info
	void serverIs(HiredisServerInfo server) {
		// delete existing connection
		if (NULL != context_ || server.hostname_.empty()) {
			redisFree(context_);
		}
		if (server.hostname_.empty()) {
			// nothing to do
			return;
		}
		// set new server info
		server_info_ = server;
		// connect to new server
		auto tmp_context = redisConnectWithTimeout(server.hostname_.c_str(), server.port_, server.timeout_);
        if (NULL == tmp_context) {
        	throw(runtime_error("Could not allocate redis context."));
        }
    	if (tmp_context->err) {
			std::string err = string("Could not connect to redis server : ") + string(tmp_context->errstr);
			redisFree(tmp_context);
			throw(runtime_error(err.c_str()));
        }
    	// set context, ping server
    	context_ = tmp_context;
	}

public:
	// set expiry (ms) on an existing db key
	void keyExpiryIs(const string& key, const uint expiry_ms) {
		reply_ = (redisReply *)redisCommand(context_, "PEXPIRE %s %s", key.c_str(), std::to_string(expiry_ms).c_str());
		// NOTE: write commands dont check for write errors.
		freeReplyObject((void*)reply_);
	}

	bool getCommandIs(const string &cmd_mssg) {
		reply_ = (redisReply *)redisCommand(context_, "GET %s", cmd_mssg.c_str());
		if (NULL == reply_ || REDIS_REPLY_ERROR == reply_->type) {
			throw(runtime_error("Server error in fetching data!"));
			//TODO: indicate what error
		}
		if (REDIS_REPLY_NIL == reply_->type) {
			// cout << "\nNo data on server.. Missing key?";
			return false;
		}
		return true;
	}

	bool getCommandIs(const string &cmd_mssg, string &ret_string) {
		reply_ = (redisReply *)redisCommand(context_, "GET %s", cmd_mssg.c_str());
		if (NULL == reply_ || REDIS_REPLY_ERROR == reply_->type) {
			throw(runtime_error("Server error in fetching data!"));
			//TODO: indicate what error
		}
		if (REDIS_REPLY_NIL == reply_->type) {
			// cout << "\nNo data on server.. Missing key?";
			return false;
		}
		ret_string = reply_->str;
		return true;
	}

	bool getCommandIs(const string &cmd_mssg, double &ret_double) {
		reply_ = (redisReply *)redisCommand(context_, "GET %s", cmd_mssg.c_str());
		if (NULL == reply_ || REDIS_REPLY_ERROR == reply_->type) {
			throw(runtime_error("Server error in fetching data!"));
			//TODO: indicate what error
		}
		if (REDIS_REPLY_NIL == reply_->type) {
			// cout << "\nNo data on server.. Missing key?";
			return false;
		}
		ret_double = std::stod(reply_->str);
		return true;
	}

	void setCommandIs(const string &cmd_mssg, const string &data_mssg) {
		reply_ = (redisReply *)redisCommand(context_, "SET %s %s", cmd_mssg.c_str(), data_mssg.c_str());
		// NOTE: set commands dont check for write errors.
      	freeReplyObject((void*)reply_);
	}

	void setCommandBatch(const std::vector<string> &cmd_mssg_vec, const std::vector<std::array<double, 7>> &data_mssg_vec, const int &n_messages)
	{
		string data_mssg_indiv;
		string batch_mssg = "";

		if(cmd_mssg_vec.size() != n_messages || data_mssg_vec.size() != n_messages)
		{
			throw(runtime_error("Not the same number of messages and keys as the expected one in setCommandBatch\n"));
		}

		// set the commands
		int cmd = 0;
		for(int i=0; i<n_messages; i++)
		{
			hDoubleArraytoStringArrayJSON(data_mssg_vec[i], 7, data_mssg_indiv);
			// batch_mssg += "SET " + cmd_mssg_vec[i] + " " + data_mssg_indiv + "\r\n";
			redisAppendCommand(context_,"SET %s %s", cmd_mssg_vec[i].c_str(), data_mssg_indiv.c_str());
			++cmd;
		}
	    /* Read (and ignore) the replies */
		while ( cmd-- > 0 )
		{
			int r = redisGetReply(context_, (void **) &reply_ );
			if ( r == REDIS_ERR ) { printf("Error\n"); exit(-1); }
			CHECK(reply_);        
			freeReplyObject(reply_);
		}
	}

	void setGetBatchCommands(const std::vector<string> &cmd_mssg_vec, 
			std::array<double, 7> &get_data_mssg_cmd_torques,
			const Eigen::MatrixXd &set_data_mssg_massmatrix,
			const std::vector<std::array<double, 7>> &set_data_mssg_vec)
	{
		string data_mssg_indiv;
		string batch_mssg = "";

		int n_messages = set_data_mssg_vec.size() + 2;

		if(cmd_mssg_vec.size() != n_messages)
		{
			throw(runtime_error("Not the same number of messages and keys as the expected one in setCommandBatch\n"));
		}

		// get command
		int cmd = 0;
		redisAppendCommand(context_,"GET %s", cmd_mssg_vec[0].c_str());
		++cmd;

		// set commands
		hEigentoStringArrayJSON(set_data_mssg_massmatrix, data_mssg_indiv);
		redisAppendCommand(context_,"SET %s %s", cmd_mssg_vec[1].c_str(), data_mssg_indiv.c_str());
		++cmd;
		for(int i=2; i < n_messages; i++)
		{
			hDoubleArraytoStringArrayJSON(set_data_mssg_vec[i-2], 7, data_mssg_indiv);
			redisAppendCommand(context_,"SET %s %s", cmd_mssg_vec[i].c_str(), data_mssg_indiv.c_str());
			++cmd;
		}
	    /* Read (and process) the repliy to the get command */
		int r = redisGetReply(context_, (void **) &reply_ );
		if ( r == REDIS_ERR ) { printf("Error\n"); exit(-1); }	    
		CHECK(reply_);   
		if(!hDoubleArrayFromStringArrayJSON(get_data_mssg_cmd_torques, 7, reply_->str)) {
			throw(runtime_error("Could not deserialize custom string to eigen data!"));
		}
		cmd--;

	    /* Read (and ignore) the replies to the set commands */
		while ( cmd-- > 0 )
		{
			int r = redisGetReply(context_, (void **) &reply_ );
			if ( r == REDIS_ERR ) { printf("Error\n"); exit(-1); }
			CHECK(reply_);        
			freeReplyObject(reply_);
		}
	}

	// write raw eigen vector
	template<typename Derived>
	void setEigenMatrixDerived(const string &cmd_mssg, const Eigen::MatrixBase<Derived> &set_mat) {
		string data_mssg;
		// serialize
		hEigentoStringArrayJSON(set_mat, data_mssg); //this never fails
		// set to server
		setCommandIs(cmd_mssg, data_mssg);
	}

    // write raw eigen vector, but in custom string format
    template<typename Derived>
    void setEigenMatrixDerivedString(const string &cmd_mssg, const Eigen::MatrixBase<Derived> &set_mat) {
    	string data_mssg;
		// serialize
		hEigenToStringArrayCustom(set_mat, data_mssg);
		// set to server
		setCommandIs(cmd_mssg, data_mssg);
    }

	// read raw eigen vector:
	template<typename Derived>
	void getEigenMatrixDerived(const string &cmd_mssg, Eigen::MatrixBase<Derived> &ret_mat) {
		auto success = getCommandIs(cmd_mssg);
		// deserialize
		if(success && !hEigenFromStringArrayJSON(ret_mat, reply_->str)) {
			throw(runtime_error("Could not deserialize json to eigen data!"));
		}
		freeReplyObject((void*)reply_);	
	}

	// read raw eigen vector, but from a custom string rather than from json
	template<typename Derived>
	void getEigenMatrixDerivedString(const string &cmd_mssg, Eigen::MatrixBase<Derived> &ret_mat) {
		auto success = getCommandIs(cmd_mssg);
		// deserialize
		if(success && !hEigenFromStringArrayCustom(ret_mat, reply_->str)) {
			throw(runtime_error("Could not deserialize custom string to eigen data!"));
		}
		freeReplyObject((void*)reply_);	
	}

	// write raw double array of length n
	template<int n>
	void setDoubleArray(const string &cmd_mssg, const double (&set_array)[n], const int &array_length) {
		string data_mssg;
		// serialize
		hDoubleArraytoStringArrayJSON(set_array, array_length, data_mssg); //this never fails
		// set to server
		setCommandIs(cmd_mssg, data_mssg);
	}

	// read double array of length n
	template<int n>
	void getDoubleArray(const string &cmd_mssg, double (&ret_array)[n], const int &array_length) {
		auto success = getCommandIs(cmd_mssg);
		// deserialize
		if(success && !hDoubleArrayFromStringArrayJSON(ret_array, array_length, reply_->str)) {
			throw(runtime_error("Could not deserialize json to double array data!"));
		}
		freeReplyObject((void*)reply_);	
	}

	// write raw double array of length n
	template<long unsigned int n>
	void setDoubleArray(const string &cmd_mssg, const std::array<double, n> &set_array, const int &array_length) {
		string data_mssg;
		// serialize
		hDoubleArraytoStringArrayJSON(set_array, array_length, data_mssg); //this never fails
		// set to server
		setCommandIs(cmd_mssg, data_mssg);
	}

	// read double array of length n
	template<long unsigned int n>
	void getDoubleArray(const string &cmd_mssg, std::array<double, n> &ret_array, const int &array_length) {
		auto success = getCommandIs(cmd_mssg);
		// deserialize
		if(success && !hDoubleArrayFromStringArrayJSON(ret_array, array_length, reply_->str)) {
			throw(runtime_error("Could not deserialize json to double array data!"));
		}
		freeReplyObject((void*)reply_);	
	}

public: // server connectivity tools
	void ping() {
		// PING server to make sure things are working..
        reply_ = (redisReply *)redisCommand(context_,"PING");
        cout<<"\n\nDriver Redis Task : Pinged Redis server. Reply is, "<<reply_->str<<"\n";
        freeReplyObject((void*)reply_);
	}

public:
	HiredisServerInfo server_info_;
	redisContext *context_;
    redisReply *reply_;
	//TODO: decide if needed. Currently, we throw up if server disconnects
	//bool connected_;
protected:
	template<typename Derived>
	bool hEigentoStringArrayJSON(const Eigen::MatrixBase<Derived> &, std::string &);

	template<int n>
	bool hDoubleArraytoStringArrayJSON(const double (&)[n], const int &, std::string &);

	template<long unsigned int n>
	bool hDoubleArraytoStringArrayJSON(const std::array<double, n> &, const int &, std::string &);
	
	template<typename Derived>
	bool hEigenFromStringArrayJSON(Eigen::MatrixBase<Derived> &, const std::string &);

	template<int n>
	bool hDoubleArrayFromStringArrayJSON(double (&)[n], const int &, const std::string &);

	template<long unsigned int n>
	bool hDoubleArrayFromStringArrayJSON(std::array<double, n> &, const int &, const std::string &);

	template<typename Derived>
	bool hEigenToStringArrayCustom(const Eigen::MatrixBase<Derived> &, std::string &);

	template<typename Derived>
	bool hEigenFromStringArrayCustom(Eigen::MatrixBase<Derived> &, const std::string &);

private:
	// internal function. prepends robot name to command
	static inline string robotCmd(string robot_name, string cmd) {
		return robot_name + ":" + cmd;
	}
};

//Implementation must be part of header for compile time template specialization
template<typename Derived>
bool CDatabaseRedisClient::hEigentoStringArrayJSON(const Eigen::MatrixBase<Derived>& x, std::string& arg_str)
{
	std::stringstream ss;
	bool row_major = true;
	if(x.cols() == 1) row_major = false; //This is a Vector!
	arg_str = "[";
	if(row_major)
	{// [1 2 3; 4 5 6] == [ [1, 2, 3], [4, 5, 6] ]
	  for(int i=0;i<x.rows();++i){
	    if(x.rows() > 1){
	      // If it is only one row, don't need the second one
	      if(i>0) arg_str.append(",[");
	      else arg_str.append("[");
	    }
	    else if(i>0) arg_str.append(",");
	    for(int j=0;j<x.cols();++j){
	      ss<<x(i,j);
	      if(j>0) arg_str.append(",");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	    if(x.rows() > 1){
	      // If it is only one row, don't need the second one
	      arg_str.append("]");
	    }
	  }
	  arg_str.append("]");
	}
	else
	{// [1 2 3; 4 5 6] == 1 4 2 5 3 6
	  for(int j=0;j<x.cols();++j){
	    if(x.cols() > 1){
	      // If it is only one row, don't need the second one
	      if(j>0) arg_str.append(",[");
	      else arg_str.append("[");
	    }
	    else if(j>0) arg_str.append(",");
	    for(int i=0;i<x.rows();++i){
	      ss<<x(i,j);
	      if(i>0) arg_str.append(",");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	    if(x.cols() > 1){
	      // If it is only one row, don't need the second one
	      arg_str.append("]");
	    }
	  }
	  arg_str.append("]");
	}
	return true;
}

template<long unsigned int n>
bool CDatabaseRedisClient::hDoubleArraytoStringArrayJSON(const std::array<double, n> &x, const int& array_length, std::string& arg_str)
{
	std::stringstream ss;
	arg_str = "[";
	for(int i=0; i<array_length; ++i)
	{
		if(i>0) 
		{
			arg_str.append(",");
		}
		ss<<x[i];
		arg_str.append(ss.str());
		ss.str(std::string());
	}
	arg_str.append("]");
	return true;
}

template<typename Derived>
bool CDatabaseRedisClient::hEigenFromStringArrayJSON(Eigen::MatrixBase<Derived>& x, const std::string &arg_str)
{
	Json::Value jval;
    Json::Reader json_reader;
    if(!json_reader.parse(arg_str,jval))
    { return false; }

	if(!jval.isArray()) return false; //Must be an array..
	unsigned int nrows = jval.size();
	if(nrows < 1) return false; //Must have elements.

	bool is_matrix = jval[0].isArray();
	if(!is_matrix)
	{
	  x.setIdentity(nrows,1);//Convert it into a vector.
	  for(int i=0;i<nrows;++i) x(i,0) = jval[i].asDouble();
	}
	else
	{
	  unsigned int ncols = jval[0].size();
	  x.setIdentity(nrows,ncols);
	  if(ncols < 1) return false; //Must have elements.
	  for(int i=0;i<nrows;++i){
	    if(ncols != jval[i].size()) return false;
	    for(int j=0;j<ncols;++j)
	      x(i,j) = jval[i][j].asDouble();
	  }
	}
	return true;
}

template<long unsigned int n>
bool CDatabaseRedisClient::hDoubleArrayFromStringArrayJSON(std::array<double, n> &x, const int& array_length, const std::string &arg_str)
{
	Json::Value jval;
	Json::Reader json_reader;
	if(!json_reader.parse(arg_str,jval))
		{ return false; }

	if(!jval.isArray()) return false; //Must be an array..
	unsigned int nrows = jval.size();
	if(nrows < 1) return false; //Must have elements.

	bool is_matrix = jval[0].isArray();
	if(!is_matrix)
	{
		if(nrows != array_length)
		{
			std::cout << "WARNING : JSON array size inconsistent with double array size\n";
			return false;
		}
		for(int i=0; i<array_length; ++i) 
		{
			x[i] = jval[i].asDouble();
		}
	}
	else
	{
		std::cout << "WARNING : Trying to read a JSON Matrix to a double array\n";
		return false;
	}
	return true;
}

template<typename Derived>
bool CDatabaseRedisClient::hEigenToStringArrayCustom(const Eigen::MatrixBase<Derived>& x, std::string& arg_str)
{
	std::stringstream ss;
	bool row_major = true;
	if(x.cols() == 1) row_major = false; //This is a Vector!
	arg_str = "";
	if(row_major)
	{// [1 2 3; 4 5 6] == '1 2 3; 4 5 6' without the ''
	  for(int i=0;i<x.rows();++i){
	    if(i>0) { arg_str.append("; "); }
	    for(int j=0;j<x.cols();++j){
	      ss << std::setprecision(12) << std::fixed << x(i,j);
	      if(j>0) arg_str.append(" ");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	  }
	}
	else
	{// [1 2 3; 4 5 6] == 1 4; 2 5; 3 6
	  for(int j=0;j<x.cols();++j){
	    if(j>0) arg_str.append("; ");
	    for(int i=0;i<x.rows();++i){
	      ss << std::setprecision(12) << std::fixed << x(i,j);
	      if(i>0) arg_str.append(" ");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	  }
	}
	return true;
}

template<typename Derived>
bool CDatabaseRedisClient::hEigenFromStringArrayCustom(Eigen::MatrixBase<Derived>& x, const std::string &arg_str)
{
	string copy_str = arg_str;
	unsigned int nrows = std::count(arg_str.begin(), arg_str.end(), ';') + 1;
	unsigned int ncols = (std::count(arg_str.begin(), arg_str.end(), ' ') + 1)/nrows;
	std::replace(copy_str.begin(), copy_str.end(), ';', ' ');

    std::stringstream ss(copy_str);
    
    if(nrows < 1) return false; //Must have elements.

	bool is_matrix = (nrows > 1);
	if(!is_matrix)
	{
	  x.setIdentity(ncols,1);//Convert it into a vector.
	  for(int i=0;i<ncols;++i) {
	  	std::string val;
		ss >> val;
	  	x(i,0) = std::stod(val);
	  }
	}
	else
	{
	  if(nrows < 1) return false; //Must have elements.
	  for(int i=0;i<nrows;++i){
	    for(int j=0;j<ncols;++j) {
	    	std::string val;
			ss >> val;
			x(i,j) = std::stod(val);
		}
	  }
	}
	return true;
}

} // namespace Franka
} // namespace Sai

#endif //CDATABASEREDISCLIENT_H