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
		std::map<std::string, int> m_mapCount;
		std::map<std::string, int> m_mapTime;
		std::map<std::string, std::chrono::steady_clock::time_point> m_mapCurrent;
	public:

		static void Start(const std::string& name)
		{
			GetTimer ().m_mapCount[name]++;
			GetTimer().m_mapCurrent[name] = std::chrono::steady_clock::now();
		}

		static void Stop(const std::string& name)
		{
			GetTimer().m_mapTime[name] += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - GetTimer().m_mapCurrent[name]).count();
		}

		static Timer& GetTimer()
		{
			static Timer tm;
			return tm;
		}

		static void PrintTimers()
		{
			for (auto p : GetTimer().m_mapCount)
			{
				std::cout << "Timer " << p.first << " was called " << p.second << " times." << std::endl;
				std::cout << "    Total time:     " << GetTimer().m_mapTime[p.first] << " ms " << std::endl;
				std::cout << "    Average time:   " << GetTimer().m_mapTime[p.first] / p.second << " ms " << std::endl;
			}
		}
	};

	class AutoTimer
	{
	public:
		AutoTimer(const std::string sName) : m_sName (sName) { Timer::GetTimer().Start(sName); }
		~AutoTimer () { Timer::GetTimer().Stop(m_sName); }

	private:
		std::string m_sName;
	};

//	std::map<std::string, int> Timer::m_mapCount = {};
//	std::map<std::string, int> Timer::m_mapTime = {};
//	std::map<std::string, std::chrono::steady_clock::time_point> Timer::m_mapCurrent = {};
}

#define START_TIMER(name) geomlib::Timer::Start(name); geomlib::Timer::GetTimer();
#define STOP_TIMER(name) geomlib::Timer::Stop(name);

#define START_AUTO_TIMER(name) AutoTimer _tm##name (#name);