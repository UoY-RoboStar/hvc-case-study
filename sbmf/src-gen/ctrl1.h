#ifndef ROBOCALC_CONTROLLERS_CTRL1_H_
#define ROBOCALC_CONTROLLERS_CTRL1_H_

#include "RP1.h"
#include "RoboCalcAPI/Controller.h"
#include "DataTypes.h"

#include "Watchdog.h"

class ctrl1: public robocalc::Controller 
{
public:
	ctrl1(RP1& _platform) : platform(&_platform){};
	ctrl1() : platform(nullptr){};
	
	~ctrl1() = default;
	
	void Execute()
	{
		watchdog.execute();
	}
	
	struct Channels
	{
		ctrl1& instance;
		Channels(ctrl1& _instance) : instance(_instance) {}
		
		EventBuffer* tryEmitInt_ActualHV(void* sender, std::tuple<double> args)
		{
			if(instance.watchdog.canReceiveInt_ActualHV(args))
				instance.watchdog.int_ActualHV_in.trigger(sender, args);
				
			return &instance.watchdog.int_ActualHV_in;
		}
		
		EventBuffer* tryEmitInt_DisableHV(void* sender, std::tuple<> args)
		{
			int_DisableHV_in.trigger(sender, args);
			return &int_DisableHV_in;
		}
		
		EventBuffer* tryEmitInt_underLimit(void* sender, std::tuple<double> args)
		{
			if(instance.watchdog.canReceiveInt_underLimit(args))
				instance.watchdog.int_underLimit_in.trigger(sender, args);
				
			return &instance.watchdog.int_underLimit_in;
		}
		
		EventBuffer* tryEmitInt_overLimit(void* sender, std::tuple<double> args)
		{
			if(instance.watchdog.canReceiveInt_overLimit(args))
				instance.watchdog.int_overLimit_in.trigger(sender, args);
				
			return &instance.watchdog.int_overLimit_in;
		}
		
		EventBuffer* tryEmitExt_pow24VStatus(void* sender, std::tuple<Power> args)
		{
			if(instance.watchdog.canReceiveExt_pow24VStatus(args))
				instance.watchdog.ext_pow24VStatus_in.trigger(sender, args);
				
			return &instance.watchdog.ext_pow24VStatus_in;
		}
		
		struct Int_DisableHV_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} int_DisableHV_in;
	};
	
	Channels channels{*this};
	
	RP1* platform;
	Watchdog_StateMachine<ctrl1> watchdog{*platform, *this, &watchdog};
};

#endif
