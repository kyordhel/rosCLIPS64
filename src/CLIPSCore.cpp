#include "CLIPSCore.h"
#include<iostream>
#include<cstdlib>
#include<vector>
#include <clips/evaluatn.h>


CLIPSCore::CLIPSCore(){
	this->environment = CreateEnvironment();
	this->routerName = (char*)"router";

	std::vector<std::string> routers = { this->routerName,
		"stdout", "werror", "stderr", "stdwarn" };

	for(auto& r : routers){
		EnvAddRouter(this->environment,
					(char*)r.c_str(), 10,
					&CLIPSCore::queryFunction,
					&CLIPSCore::printFunction,
					&CLIPSCore::getcFunction,
					&CLIPSCore::ungetcFunction,
					&CLIPSCore::exitFunction);
		EnvActivateRouter(this->environment, (char*)r.c_str());
	}
	EnvDeactivateRouter(this->environment, (char*)"Eval-1");
	EnvDeactivateRouter(this->environment, (char*)"t");
	EnvDeleteRouter(this->environment, (char*)"t");
}

CLIPSCore::~CLIPSCore(){
	// std::cout << "~CLIPSCORE()" << std::endl;
	// DeallocateEnvironmentData();
	// std::cout << "Deallocated environment data" << std::endl;
	if(this->environment){
		DestroyEnvironment(this->environment);
		// std::cout << "Environment destroyed" << std::endl;
		this->environment = NULL;
	}
}

bool CLIPSCore::initKDB(const std::string& path, bool flag, int i){
	if( CLIPSCore::getInstance().load(path) )
		return true;
	return false;
}

void CLIPSCore::resetCLIPS(bool flag){
	CLIPSCore::getInstance().reset();
}

void CLIPSCore::factCLIPS(bool flag){
	CLIPSCore::getInstance().listFacts();
}

void CLIPSCore::ruleCLIPS(bool flag){
	CLIPSCore::getInstance().listRules();
}

void CLIPSCore::runCLIPS(bool flag){
	CLIPSCore::getInstance().run(-1);
}

bool CLIPSCore::strQueryKDB(const std::string& query){
	return CLIPSCore::getInstance().eval(query);
}




int CLIPSCore::queryFunction(void* environment, char *ln){
	std::string sln(ln);
	if (
		!sln.compare( CLIPSCore::getInstance().routerName ) ||
		!sln.compare( "stdout" ) ||
		!sln.compare( "Eval-1" ) ||
		!sln.compare( "stderr" ) ||
		!sln.compare( "stdwrn" ) ||
		!sln.compare( "werror" )
	)
		return true;
	printf("Query: [%s]\n", ln);
	return false;
}

int CLIPSCore::printFunction(void* environment, char *ln, char *str){
	std::string sln(ln);
	/*
	printf("print: %s -> [%s]\n", ln, str);
	if( !sln.compare( CLIPSCore::getInstance().routerName ) ||
		!sln.compare( "stdout" ) )
			CLIPSCore::getInstance().ssCLIPSout << str;
	else if( !sln.compare( "stderr" ) )
			CLIPSCore::getInstance().ssCLIPSerr << str;
	else if( !sln.compare( "stdwrn" ) )
			CLIPSCore::getInstance().ssCLIPSwrn << str;
	else
		return 0;
	return 1;
	*/
	if( !sln.compare( "stderr" ) || !sln.compare( "werror" ) )
		CLIPSCore::getInstance().ssCLIPSerr << str;
	else if( !sln.compare( "stdwrn" ) )
		CLIPSCore::getInstance().ssCLIPSwrn << str;
	else if(
			CLIPSCore::getInstance().routerName ||
			!sln.compare( "stdout" ) || !sln.compare( "Eval-1" )
		)
		CLIPSCore::getInstance().ssCLIPSout << str;
	return 1;
}

int CLIPSCore::getcFunction(void* environment, char *ln){
	// printf("getcFunction(%s) -> %c [%s]\n", ln, CLIPSCore::getInstance().ssCLIPSin.peek(), CLIPSCore::getInstance().ssCLIPSin.str().c_str());
	return CLIPSCore::getInstance().ssCLIPSin.get();
}

int CLIPSCore::ungetcFunction(void* environment, int chr, char *ln){
	CLIPSCore::getInstance().ssCLIPSin.unget();
	// printf("ungetcFunction(%s) <- %c [%s]\n", ln, CLIPSCore::getInstance().ssCLIPSin.peek(), CLIPSCore::getInstance().ssCLIPSin.str().c_str());
	return 1;
}

int CLIPSCore::exitFunction(void* environment, int code){
	// printf("exitFunction()\n");
	return 1;
}





void CLIPSCore::clear(){
	EnvClear(this->environment);
}

bool CLIPSCore::eval(const std::string& s){
	// CLIPSValue *cv = NULL;
	this->ssCLIPSin.str(s);
	DATA_OBJECT cv;
	return !EnvEval(this->environment,
		(char*) s.c_str(),
		&cv
	);
}

void CLIPSCore::listFacts(){
	EnvFacts(this->environment,
		this->routerName,
		NULL, -1, -1, -1);
}

void CLIPSCore::listRules(){
	EnvListDefrules(this->environment,
		this->routerName,
		NULL);
}

bool CLIPSCore::load(const std::string& path){
	char* cpath = (char*)path.c_str();
	cpath = realpath(cpath, NULL);
	if( !cpath ) return 0;
	return EnvLoad(this->environment, cpath) != 0;
}

void CLIPSCore::reset(){
	EnvReset(this->environment);
}

long long CLIPSCore::run(long long limit){
	return EnvRun(this->environment, limit);
}

std::string CLIPSCore::readOut(bool flush){
	std::string s(this->ssCLIPSout.str());
	if(flush) this->ssCLIPSout.str(std::string());
	return s;
}

std::string CLIPSCore::readErr(bool flush){
	std::string s(this->ssCLIPSerr.str());
	if(flush) this->ssCLIPSerr.str(std::string());
	return s;
}

std::string CLIPSCore::readWrn(bool flush){
	std::string s(this->ssCLIPSwrn.str());
	if(flush) this->ssCLIPSwrn.str(std::string());
	return s;
}







CLIPSCore& CLIPSCore::getInstance(){
	static CLIPSCore singleton; // Will be instantiated on first use
	return singleton;
}

