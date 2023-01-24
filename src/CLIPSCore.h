// #include<iostream>
// #include<cstdlib>
#include<string>
#include<sstream>
// #include<vector>

// #include<stdlib.h>

#include <clips/clips.h>
// #include <clips/evaluatn.h>


#undef ROUTER_DATA // We are redefining this from clips.h
#define ROUTER_DATA USER_ENVIRONMENT_DATA + 1


class CLIPSCore {
public:
	/**
	 * Gets the singleton instance
	 * @return The singleton instance
	 */
	static CLIPSCore& getInstance();

	/**
	 * Function to load a clp file (knowledgebase)
	 * @param  path Path of the clp file to load
	 * @param  flag Unknown
	 * @param  i    Unknown
	 * @return      true if the file was loaded successfully, false otherwise
	 */
	static bool initKDB(const std::string& path, bool flag, int i);

	/**
	 * Resets CLIPS
	 */
	static void resetCLIPS(bool flag);

	/**
	 * Prints the list of facts to the router
	 */
	static void factCLIPS(bool flag);

	/**
	 * Prints the list of rules to the router
	 */
	static void ruleCLIPS(bool flag);

	/**
	 * Starts executing CLIPS
	 */
	static void runCLIPS(bool flag);

	/**
	 * Queries to the knowledgebase. It executes the query
	 * using the Eval function. Results (if any) are printed to
	 * the router.
	 *
	 * @param  query The query to perform in CLIPS language
	 * @return       True if the query was executed successfully,
	 *               false otherwise.
	 */
	static bool strQueryKDB(const std::string& query);



public: // Singleton required (C++ 11)
	CLIPSCore(CLIPSCore const&)      = delete; // Singleton copy constructor
	void operator=(CLIPSCore const&) = delete; // Singleton assignment op

public: // Instance methods
	void clear();
	bool eval(const std::string& s);
	bool load(const std::string& path);
	void listFacts();
	void listRules();
	void reset();
	long long run(long long limit);

	std::string readOut(bool flush=true);
	std::string readErr(bool flush=true);
	std::string readWrn(bool flush=true);


private:
	void* environment;
	char* routerName;
	std::stringstream ssCLIPSin;
	std::stringstream ssCLIPSout;
	std::stringstream ssCLIPSerr;
	std::stringstream ssCLIPSwrn;

private:
	CLIPSCore();
	~CLIPSCore();

private:
	// prototypes for router handling functions. ln is the logical name
	static int queryFunction( void* environment, char *ln);
	static int printFunction( void* environment, char *ln, char *str);
	static int getcFunction(  void* environment, char *ln);
	static int ungetcFunction(void* environment, int chr, char *ln);
	static int exitFunction(  void* environment, int code);
};
