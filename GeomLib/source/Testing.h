#pragma once
#include "Segment.h"
#include "Line.h"
#include "Ray.h"
#include <fstream>
#include <string>
#include <vector>
#include <map>

#define TESTING_SECTION_OPEN std::ofstream fout("test_log.txt");

#define TEST(name) fout << "Test \"""" << name << "\""" results:" << std::endl;

#define SUBTEST_EQ(subname, arg1, arg2) fout << "	Subtest \"""" << subname << "\""": ";	\
	try {																					\
		if (CheckEquality<decltype(arg1), decltype(arg2)>::Answer(arg1, arg2))				\
			fout << "OK" << std::endl;														\
		else fout << "WA" << std::endl;														\
	}																						\
	catch (std::exception e) {																\
		fout << "RTE with message " << std::endl << e.what() << std::endl;					\
	};	

#define SUBTEST_UNEQ(subname, arg1, arg2) fout << "	Subtest \"""" << subname << "\""": ";	\
	try {																					\
		if (!CheckEquality<decltype(arg1), decltype(arg2)>::Answer(arg1, arg2))				\
			fout << "OK" << std::endl;														\
		else fout << "WA" << std::endl;														\
	}																						\
	catch (std::exception e) {																\
		fout << "RTE with message " << std::endl << e.what() << std::endl;					\
	};	

#define SUBTEST_ASSERT(subname, arg1) fout << "	Subtest \"""" << subname << "\""": ";		\
	try {																					\
		if (CheckEquality<decltype(arg1), bool>::Answer(arg1, true))						\
			fout << "OK" << std::endl;														\
		else fout << "WA" << std::endl;														\
	}																						\
	catch (std::exception e) {																\
		fout << "RTE with message " << std::endl << e.what() << std::endl;					\
	};	

#define TESTING_SECTION_CLOSE fout.close();

template <class T, class S>
class CheckEquality {
public:
	static bool Answer(T arg1, S arg2) {
		if (std::is_arithmetic<T>::value && std::is_arithmetic<S>::value) return arg1 == arg2;
		return false;
	}
};

template <class T>
class CheckEquality<T, T> {
public:
	static bool Answer(T arg1, T arg2) {
		return arg1 == arg2;
	}
};
