#ifndef ROBOCALC_CONTROLLERS_CTRL0_H_
#define ROBOCALC_CONTROLLERS_CTRL0_H_

#include "RP1.h"
#include "RoboCalcAPI/Controller.h"
#include "DataTypes.h"

#include "State_machine.h"

class ctrl0: public robocalc::Controller 
{
public:
	ctrl0(RP1& _platform) : platform(&_platform){};
	ctrl0() : platform(nullptr){};
	
	~ctrl0() = default;
	
	void Execute()
	{
		state_machine.execute();
	}
	
	struct Channels
	{
		ctrl0& instance;
		Channels(ctrl0& _instance) : instance(_instance) {}
		
		EventBuffer* tryEmitInt_DisableHV(void* sender, std::tuple<> args)
		{
			if(instance.state_machine.canReceiveInt_DisableHV(args))
				instance.state_machine.int_DisableHV_in.trigger(sender, args);
				
			return &instance.state_machine.int_DisableHV_in;
		}
		
		EventBuffer* tryEmitInt_ActualHV(void* sender, std::tuple<double> args)
		{
			int_ActualHV_in.trigger(sender, args);
			return &int_ActualHV_in;
		}
		
		EventBuffer* tryEmitInt_underLimit(void* sender, std::tuple<double> args)
		{
			int_underLimit_in.trigger(sender, args);
			return &int_underLimit_in;
		}
		
		EventBuffer* tryEmitInt_overLimit(void* sender, std::tuple<double> args)
		{
			int_overLimit_in.trigger(sender, args);
			return &int_overLimit_in;
		}
		
		EventBuffer* tryEmitExt_setPoint(void* sender, std::tuple<double> args)
		{
			if(instance.state_machine.canReceiveExt_setPoint(args))
				instance.state_machine.ext_setPoint_in.trigger(sender, args);
				
			return &instance.state_machine.ext_setPoint_in;
		}
		
		EventBuffer* tryEmitInt_pwmSignal(void* sender, std::tuple<Power> args)
		{
			int_pwmSignal_in.trigger(sender, args);
			return &int_pwmSignal_in;
		}
		
		EventBuffer* tryEmitExt_pow24VStatus(void* sender, std::tuple<Power> args)
		{
			if(instance.state_machine.canReceiveExt_pow24VStatus(args))
				instance.state_machine.ext_pow24VStatus_in.trigger(sender, args);
				
			return &instance.state_machine.ext_pow24VStatus_in;
		}
		
		EventBuffer* tryEmitCurrentState(void* sender, std::tuple<State> args)
		{
			currentState_in.trigger(sender, args);
			return &currentState_in;
		}
		
		struct Int_ActualHV_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<double> _args;
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
			
			void trigger(void* sender, std::tuple<double> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} int_ActualHV_in;
		struct Int_underLimit_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<double> _args;
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
			
			void trigger(void* sender, std::tuple<double> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} int_underLimit_in;
		struct Int_overLimit_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<double> _args;
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
			
			void trigger(void* sender, std::tuple<double> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} int_overLimit_in;
		struct Int_pwmSignal_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<Power> _args;
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
			
			void trigger(void* sender, std::tuple<Power> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} int_pwmSignal_in;
		struct CurrentState_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<State> _args;
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
			
			void trigger(void* sender, std::tuple<State> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} currentState_in;
		struct Ext_setPoint_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<double> _args;
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
			
			void trigger(void* sender, std::tuple<double> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} ext_setPoint_in;
		
	};
	
	Channels channels{*this};
	
	RP1* platform;
	State_machine_StateMachine<ctrl0> state_machine{*platform, *this, &state_machine};
};

#endif
