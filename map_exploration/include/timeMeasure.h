#ifndef TIME_MEASURE_H
#define TIME_MEASURE_H

class TimeMeasurer{
	private:
		struct timespec startT, finish;
		double elapsed;

	public:
		void start(){
			clock_gettime(CLOCK_MONOTONIC, &startT);
		};
		void end(){
			clock_gettime(CLOCK_MONOTONIC, &finish);
			elapsed = (finish.tv_sec - startT.tv_sec);
			elapsed += (finish.tv_nsec - startT.tv_nsec) / 1000000000.0;
		};
		std::string toString(){
			std::stringstream ss;
    		ss << elapsed;
    		return ss.str();
		};
		double getElapsedTime(){return elapsed; };
};

#endif

