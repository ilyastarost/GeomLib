#pragma once
#include <iostream>
#include <chrono>
#include <string>
#include <map>

namespace geomlib
{
	class Timer final
	{
	private:
		static std::map<std::string, int> m_mapCount;
		static std::map<std::string, int> m_mapTime;
		static std::map<std::string, std::chrono::steady_clock::time_point> m_mapCurrent;
		//Timer() = default;
	public:

		static void Start(const std::string& name)
		{
			m_mapCount[name]++;
			m_mapCurrent[name] = std::chrono::steady_clock::now();
		}

		static void Stop(const std::string& name)
		{
			m_mapTime[name] += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_mapCurrent[name]).count();
		}

		~Timer()
		{
			for (auto p : m_mapCount)
			{
				std::cout << "Timer " << p.first << " was called " << p.second << " times." << std::endl;
				std::cout << "    Total time:     " << m_mapTime[p.first] << " ms " << std::endl;
				std::cout << "    Average time:   " << m_mapTime[p.first] / p.second << " ms " << std::endl;
			}
		}
	};

	std::map<std::string, int> Timer::m_mapCount = {};
	std::map<std::string, int> Timer::m_mapTime = {};
	std::map<std::string, std::chrono::steady_clock::time_point> Timer::m_mapCurrent = {};
	Timer tm;
}

#define START_TIMER(name) geomlib::Timer::Start(name);
#define STOP_TIMER(name) geomlib::Timer::Stop(name);