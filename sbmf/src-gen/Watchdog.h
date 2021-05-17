#ifndef ROBOCALC_STATEMACHINES_WATCHDOG_H_
#define ROBOCALC_STATEMACHINES_WATCHDOG_H_

#include "RoboCalcAPI/StateMachine.h"

#ifndef ROBOCALC_THREAD_SAFE
#define THREAD_SAFE_ONLY(x)
#else
#include <mutex>
#define THREAD_SAFE_ONLY(x) x
#endif

#include "RP1.h"
#include "RoboCalcAPI/Timer.h"
#include "Functions.h"
#include "DataTypes.h"
#include <assert.h>
#include <set>
#include "disableHV.h"
#include "checkLimits.h"
#include "supplyVoltCheck.h"

using namespace robocalc;
using namespace robocalc::functions;

template<typename ControllerType>
class Watchdog_StateMachine : public StateMachine<RP1, ControllerType, Watchdog_StateMachine<ControllerType>>
{
	public:
		ControllerType& controller;
		double underLimit;
		double overLimit;
		double ActualHV;
		Power power;
	public:
		class StateMachine_S0_State_t : public StateMachine<RP1, ControllerType, Watchdog_StateMachine<ControllerType>>
		{
			public:
				ControllerType& controller;
				double& underLimit;
				double& overLimit;
				double& ActualHV;
				Power& power;
			public:
				class F0_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit F0_State_t(ControllerType& _controller, Watchdog_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						F0_State_t() = delete;
					
						void enter() override
						{
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				class Waiting_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit Waiting_State_t(ControllerType& _controller, Watchdog_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						Waiting_State_t() = delete;
					
						void enter() override
						{
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				class PowerStatusRead_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit PowerStatusRead_State_t(ControllerType& _controller, Watchdog_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						PowerStatusRead_State_t() = delete;
					
						void enter() override
						{
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				class ActualHVRead_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit ActualHVRead_State_t(ControllerType& _controller, Watchdog_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						ActualHVRead_State_t() = delete;
					
						void enter() override
						{
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				class PowerAndActualHVRead_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit PowerAndActualHVRead_State_t(ControllerType& _controller, Watchdog_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						PowerAndActualHVRead_State_t() = delete;
					
						void enter() override
						{
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				F0_State_t f0_State;
				Waiting_State_t waiting_State;
				PowerStatusRead_State_t powerStatusRead_State;
				ActualHVRead_State_t actualHVRead_State;
				PowerAndActualHVRead_State_t powerAndActualHVRead_State;
				
				enum PossibleStates
				{
					s_f0,
					s_Waiting,
					s_PowerStatusRead,
					s_ActualHVRead,
					s_PowerAndActualHVRead,
					j_i0
				} currentState = j_i0; 
		
			public:
				StateMachine_S0_State_t(RP1& _platform, ControllerType& _controller, Watchdog_StateMachine<ControllerType>* topLevel = nullptr) 
					: StateMachine<RP1, ControllerType, Watchdog_StateMachine<ControllerType>>(_platform, _controller, topLevel), controller{_controller},
					underLimit(topLevel->underLimit), overLimit(topLevel->overLimit), ActualHV(topLevel->ActualHV), power(topLevel->power),
					f0_State{_controller, topLevel}, waiting_State{_controller, topLevel}, powerStatusRead_State{_controller, topLevel}, actualHVRead_State{_controller, topLevel}, powerAndActualHVRead_State{_controller, topLevel}
					{};
				StateMachine_S0_State_t() = delete;
				~StateMachine_S0_State_t() = default;
			
			
				bool canReceiveExt_pow24VStatus(std::tuple<Power> args)
				{
					if(blockedByExt_pow24VStatus != nullptr)
						if(blockedByExt_pow24VStatus->getSender() == this)
							return false;
														
					blockedByExt_pow24VStatus = nullptr;
				
					switch(currentState)
					{
						case s_f0:
						{
							
							return false;
						}
						case s_Waiting:
						{
							return true;
							
							return false;
						}
						case s_PowerStatusRead:
						{
							
							return false;
						}
						case s_ActualHVRead:
						{
							return true;
							
							return false;
						}
						case s_PowerAndActualHVRead:
						{
							
							return false;
						}
						default:
						{
							return false;
						}
					}
				}
		
				EventBuffer* blockedByExt_pow24VStatus = nullptr;
																	
				inline void tryEmitExt_pow24VStatus(std::tuple<Power> args)
				{
					blockedByExt_pow24VStatus = controller.channels.tryEmitExt_pow24VStatus(this, args);
				}
				
				struct Ext_pow24VStatus_t : public EventBuffer
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
				} ext_pow24VStatus_in;
				
				bool canReceiveInt_ActualHV(std::tuple<double> args)
				{
					if(blockedByInt_ActualHV != nullptr)
						if(blockedByInt_ActualHV->getSender() == this)
							return false;
														
					blockedByInt_ActualHV = nullptr;
				
					switch(currentState)
					{
						case s_f0:
						{
							
							return false;
						}
						case s_Waiting:
						{
							return true;
							
							return false;
						}
						case s_PowerStatusRead:
						{
							return true;
							
							return false;
						}
						case s_ActualHVRead:
						{
							
							return false;
						}
						case s_PowerAndActualHVRead:
						{
							
							return false;
						}
						default:
						{
							return false;
						}
					}
				}
		
				EventBuffer* blockedByInt_ActualHV = nullptr;
																	
				inline void tryEmitInt_ActualHV(std::tuple<double> args)
				{
					blockedByInt_ActualHV = controller.channels.tryEmitInt_ActualHV(this, args);
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
				
			
				void execute() override
				{
					while(tryTransitions());
				
					switch(currentState)
					{
					case s_f0:
					{
						f0_State.during();
						break;
					}
					case s_Waiting:
					{
						waiting_State.during();
						break;
					}
					case s_PowerStatusRead:
					{
						powerStatusRead_State.during();
						break;
					}
					case s_ActualHVRead:
					{
						actualHVRead_State.during();
						break;
					}
					case s_PowerAndActualHVRead:
					{
						powerAndActualHVRead_State.during();
						break;
					}
						default:
							break;
					}
				}
			
				bool tryTransitions() override
				{
					switch(currentState)
					{
						case s_f0:
						{
		
							break;
						}
						case s_Waiting:
						{
							if(ext_pow24VStatus_in.getSender() != nullptr)
							{
								waiting_State.exit();
								ext_pow24VStatus_in.reset();
								powerStatusRead_State.enter();
								currentState = s_PowerStatusRead;
								return true;	
							}
							if(int_ActualHV_in.getSender() != nullptr)
							{
								waiting_State.exit();
								int_ActualHV_in.reset();
								actualHVRead_State.enter();
								currentState = s_ActualHVRead;
								return true;	
							}
		
							break;
						}
						case s_PowerStatusRead:
						{
							if((this->power) == (Power::Off))
							{
								powerStatusRead_State.exit();
								this->tryEmitInt_DisableHV(std::tuple<>{});
								f0_State.enter();
								currentState = s_f0;
								return true;	
							}
							if(int_ActualHV_in.getSender() != nullptr)
							{
								powerStatusRead_State.exit();
								int_ActualHV_in.reset();
								powerAndActualHVRead_State.enter();
								currentState = s_PowerAndActualHVRead;
								return true;	
							}
		
							break;
						}
						case s_ActualHVRead:
						{
							if((this->ActualHV) > (this->overLimit))
							{
								actualHVRead_State.exit();
								this->tryEmitInt_DisableHV(std::tuple<>{});
								f0_State.enter();
								currentState = s_f0;
								return true;	
							}
							if((this->ActualHV < (this->underLimit))
							{
								actualHVRead_State.exit();
								this->tryEmitInt_DisableHV(std::tuple<>{});
								f0_State.enter();
								currentState = s_f0;
								return true;	
							}
							if(ext_pow24VStatus_in.getSender() != nullptr)
							{
								actualHVRead_State.exit();
								ext_pow24VStatus_in.reset();
								powerAndActualHVRead_State.enter();
								currentState = s_PowerAndActualHVRead;
								return true;	
							}
		
							break;
						}
						case s_PowerAndActualHVRead:
						{
							if((this->ActualHV < (this->underLimit))
							{
								powerAndActualHVRead_State.exit();
								this->tryEmitInt_DisableHV(std::tuple<>{});
								f0_State.enter();
								currentState = s_f0;
								return true;	
							}
							if((this->ActualHV) > (this->overLimit))
							{
								powerAndActualHVRead_State.exit();
								this->tryEmitInt_DisableHV(std::tuple<>{});
								f0_State.enter();
								currentState = s_f0;
								return true;	
							}
							if((this->power) == (Power::Off))
							{
								powerAndActualHVRead_State.exit();
								this->tryEmitInt_DisableHV(std::tuple<>{});
								f0_State.enter();
								currentState = s_f0;
								return true;	
							}
		
							break;
						}
						case j_i0:
						{
							
							;
							currentState = s_Waiting;
							waiting_State.enter();
							return true;
						}
						default:
							break;
					}
						
					return false;
				}
		};
		
		class S0_State_t : public CompositeState<StateMachine_S0_State_t, RP1, ControllerType, Watchdog_StateMachine<ControllerType>>
		{
			public:
				S0_State_t(RP1& robot, ControllerType& controller, Watchdog_StateMachine<ControllerType>* topLevel) : CompositeState<StateMachine_S0_State_t, RP1, ControllerType, Watchdog_StateMachine<ControllerType>>(robot, controller, topLevel){}
				S0_State_t() = delete;
			
				void enter() override
				{
				
					CompositeState<StateMachine_S0_State_t, RP1, ControllerType, Watchdog_StateMachine<ControllerType>>::enter();
				}
			
				void exit() override
				{
		
					CompositeState<StateMachine_S0_State_t, RP1, ControllerType, Watchdog_StateMachine<ControllerType>>::exit();
				}
		
				void during() override
				{
		
					CompositeState<StateMachine_S0_State_t, RP1, ControllerType, Watchdog_StateMachine<ControllerType>>::during();
				}
		};
		S0_State_t s0_State;
		
		enum PossibleStates
		{
			s_s0,
			j_i0
		} currentState = j_i0; 

	public:
		Watchdog_StateMachine(RP1& _platform, ControllerType& _controller, Watchdog_StateMachine<ControllerType>* topLevel = nullptr) 
			: StateMachine<RP1, ControllerType, Watchdog_StateMachine<ControllerType>>(_platform, _controller, topLevel), controller{_controller},
			underLimit(0), overLimit(0), ActualHV(0), power(Power::On),
			s0_State{_platform, _controller, this}
			{};
		Watchdog_StateMachine() = delete;
		~Watchdog_StateMachine() = default;
	
	
		bool canReceiveExt_pow24VStatus(std::tuple<Power> args)
		{
			if(blockedByExt_pow24VStatus != nullptr)
				if(blockedByExt_pow24VStatus->getSender() == this)
					return false;
												
			blockedByExt_pow24VStatus = nullptr;
		
			switch(currentState)
			{
				case s_s0:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByExt_pow24VStatus = nullptr;
															
		inline void tryEmitExt_pow24VStatus(std::tuple<Power> args)
		{
			blockedByExt_pow24VStatus = controller.channels.tryEmitExt_pow24VStatus(this, args);
		}
		
		struct Ext_pow24VStatus_t : public EventBuffer
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
		} ext_pow24VStatus_in;
		
		bool canReceiveInt_ActualHV(std::tuple<double> args)
		{
			if(blockedByInt_ActualHV != nullptr)
				if(blockedByInt_ActualHV->getSender() == this)
					return false;
												
			blockedByInt_ActualHV = nullptr;
		
			switch(currentState)
			{
				case s_s0:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByInt_ActualHV = nullptr;
															
		inline void tryEmitInt_ActualHV(std::tuple<double> args)
		{
			blockedByInt_ActualHV = controller.channels.tryEmitInt_ActualHV(this, args);
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
		
	
		void execute() override
		{
			while(tryTransitions());
		
			switch(currentState)
			{
			case s_s0:
			{
				s0_State.during();
				break;
			}
				default:
					break;
			}
		}
	
		bool tryTransitions() override
		{
			switch(currentState)
			{
				case s_s0:
				{
					if((this->eT.get()) >= (10))
					{
						s0_State.exit();
						s0_State.enter();
						currentState = s_s0;
						return true;	
					}

					break;
				}
				case j_i0:
				{
					
					this->overLimit = std::get<0>(this->int_overLimit_in._args);
					this->int_overLimit_in.reset();
					this->underLimit = std::get<0>(this->int_underLimit_in._args);
					this->int_underLimit_in.reset();
					;
					currentState = s_s0;
					s0_State.enter();
					return true;
				}
				default:
					break;
			}
				
			return false;
		}
};
	
#endif
