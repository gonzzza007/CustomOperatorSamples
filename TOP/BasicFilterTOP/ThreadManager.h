#ifndef __ThreadManager__
#define __ThreadManager__

#include "TOP_CPlusPlusBase.h"

#include <mutex>
#include <atomic>
#include <condition_variable>
#include <thread>

using namespace TD;

// class Parameters;
class OP_Inputs;


enum class ThreadStatus
{
	Busy,
	Ready,
	Done
};

class ThreadManager
{
public:
	ThreadManager();

	~ThreadManager();

	void	syncParms(bool doDither, int bitsPerColor, int inWidth, int inHeight, int outWidth, int outHeight, const TD::OP_Inputs* inputs);

	void	syncBuffer(uint32_t* inBuffer, uint32_t* outBuffer);

private:
	void	threadFn();

	ThreadStatus			myStatus;

	// Out buffer resource
	uint32_t*				myInBuffer;
	uint32_t*				myOutBuffer;

	// Thread and Sync variables
	std::thread*			myThread;
	std::mutex				myBufferMutex;
	std::mutex				myParmsMutex;
	std::condition_variable myBufferCV;
	std::atomic_bool		myThreadShouldExit;

	// Parameters saved
	int						myInWidth;
	int						myInHeight;
	int						myOutWidth;
	int						myOutHeight;
	bool					myDoDither;
	int						myBitsPerColor;
};

#endif
