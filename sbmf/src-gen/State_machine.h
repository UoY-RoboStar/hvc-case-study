#ifndef ROBOCALC_STATEMACHINES_STATE_MACHINE_H_
#define ROBOCALC_STATEMACHINES_STATE_MACHINE_H_

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
class State_machine_StateMachine : public StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>
{
	public:
		ControllerType& controller;
		Power power;
	public:
		class Ramping_State_t : public robocalc::State<ControllerType>
		{
			public:
				explicit Ramping_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
					: robocalc::State<ControllerType>(_controller)
				{
				}
				
				Ramping_State_t() = delete;
			
				void enter() override
				{
					this->controller.tryEmitCurrentState(std::tuple<State>{State::Ramp});
				}
				
				void exit() override
				{
				}
				
				void during() override
				{
				}
		};
		
		class StateMachine_Init_State_t : public StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>
		{
			public:
				ControllerType& controller;
				Power& power;
			public:
				class Si0_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit Si0_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						Si0_State_t() = delete;
					
						void enter() override
						{
							this->controller. = this->controller.this->controller.overLimitF((this->controller.setPoint) + (2));
							this->controller. = this->controller.this->controller.underLimitF((this->controller.setPoint) - (2));
							this->controller.tryEmitInt_underLimit(std::tuple<double>{this->controller.underLimit});
							this->controller.tryEmitInt_overLimit(std::tuple<double>{this->controller.overLimit});
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				Si0_State_t si0_State;
				
				enum PossibleStates
				{
					s_si0,
					j_i0
				} currentState = j_i0; 
		
			public:
				StateMachine_Init_State_t(RP1& _platform, ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel = nullptr) 
					: StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>(_platform, _controller, topLevel), controller{_controller},
					power(topLevel->power),
					si0_State{_controller, topLevel}
					{};
				StateMachine_Init_State_t() = delete;
				~StateMachine_Init_State_t() = default;
			
			
			
				void execute() override
				{
					while(tryTransitions());
				
					switch(currentState)
					{
					case s_si0:
					{
						si0_State.during();
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
						case s_si0:
						{
		
							break;
						}
						case j_i0:
						{
							
							this->setPoint = std::get<0>(this->ext_setPoint_in._args);
							this->ext_setPoint_in.reset();;
							currentState = s_si0;
							si0_State.enter();
							return true;
						}
						default:
							break;
					}
						
					return false;
				}
		};
		
		class Init_State_t : public CompositeState<StateMachine_Init_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>
		{
			public:
				Init_State_t(RP1& robot, ControllerType& controller, State_machine_StateMachine<ControllerType>* topLevel) : CompositeState<StateMachine_Init_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>(robot, controller, topLevel){}
				Init_State_t() = delete;
			
				void enter() override
				{
					this->sm.tryEmitCurrentState(std::tuple<State>{State::Init});
				
					CompositeState<StateMachine_Init_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::enter();
				}
			
				void exit() override
				{
		
					CompositeState<StateMachine_Init_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::exit();
				}
		
				void during() override
				{
		
					CompositeState<StateMachine_Init_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::during();
				}
		};
		class StateMachine_Wait24Vpower_State_t : public StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>
		{
			public:
				ControllerType& controller;
				Power& power;
			public:
				class Si0_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit Si0_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						Si0_State_t() = delete;
					
						void enter() override
						{
							this->controller.platform->supplyVoltCheck();
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				class S1_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit S1_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						S1_State_t() = delete;
					
						void enter() override
						{
							this->controller.platform->disableHV(true);
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				Si0_State_t si0_State;
				S1_State_t s1_State;
				
				enum PossibleStates
				{
					s_si0,
					s_s1,
					j_i0
				} currentState = j_i0; 
		
			public:
				StateMachine_Wait24Vpower_State_t(RP1& _platform, ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel = nullptr) 
					: StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>(_platform, _controller, topLevel), controller{_controller},
					power(topLevel->power),
					si0_State{_controller, topLevel}, s1_State{_controller, topLevel}
					{};
				StateMachine_Wait24Vpower_State_t() = delete;
				~StateMachine_Wait24Vpower_State_t() = default;
			
			
			
				void execute() override
				{
					while(tryTransitions());
				
					switch(currentState)
					{
					case s_si0:
					{
						si0_State.during();
						break;
					}
					case s_s1:
					{
						s1_State.during();
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
						case s_si0:
						{
							if(((this->setPoint) != (0)) || ((this->lim) == (true)))
							{
								si0_State.exit();
								s1_State.enter();
								currentState = s_s1;
								return true;	
							}
		
							break;
						}
						case s_s1:
						{
		
							break;
						}
						case j_i0:
						{
							
							this->setPoint = std::get<0>(this->ext_setPoint_in._args);
							this->ext_setPoint_in.reset();;
							currentState = s_si0;
							si0_State.enter();
							return true;
						}
						default:
							break;
					}
						
					return false;
				}
		};
		
		class Wait24Vpower_State_t : public CompositeState<StateMachine_Wait24Vpower_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>
		{
			public:
				Wait24Vpower_State_t(RP1& robot, ControllerType& controller, State_machine_StateMachine<ControllerType>* topLevel) : CompositeState<StateMachine_Wait24Vpower_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>(robot, controller, topLevel){}
				Wait24Vpower_State_t() = delete;
			
				void enter() override
				{
					this->sm.tryEmitCurrentState(std::tuple<State>{State::Wait24Vpower});
				
					CompositeState<StateMachine_Wait24Vpower_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::enter();
				}
			
				void exit() override
				{
		
					CompositeState<StateMachine_Wait24Vpower_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::exit();
				}
		
				void during() override
				{
		
					CompositeState<StateMachine_Wait24Vpower_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::during();
				}
		};
		class StateMachine_ClosedLoop_State_t : public StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>
		{
			public:
				ControllerType& controller;
				Power& power;
			public:
				class S1_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit S1_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						S1_State_t() = delete;
					
						void enter() override
						{
							this->controller. = false;
							this->controller.platform->disableHV(true);
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				class StateMachine_S2_State_t : public StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>
				{
					public:
						ControllerType& controller;
						Power& power;
					public:
						class S0_State_t : public robocalc::State<ControllerType>
						{
							public:
								explicit S0_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
									: robocalc::State<ControllerType>(_controller)
								{
								}
								
								S0_State_t() = delete;
							
								void enter() override
								{
									this->controller.platform->checkLimits();
								}
								
								void exit() override
								{
								}
								
								void during() override
								{
								}
						};
						
						class S1_State_t : public robocalc::State<ControllerType>
						{
							public:
								explicit S1_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
									: robocalc::State<ControllerType>(_controller)
								{
								}
								
								S1_State_t() = delete;
							
								void enter() override
								{
									this->controller.setPoint = std::get<0>(this->controller.ext_setPoint_in._args);
									this->controller.ext_setPoint_in.reset();
								}
								
								void exit() override
								{
								}
								
								void during() override
								{
								}
						};
						
						S0_State_t s0_State;
						S1_State_t s1_State;
						
						enum PossibleStates
						{
							s_s0,
							s_s1,
							j_i0,
							j_j0
						} currentState = j_i0; 
				
					public:
						StateMachine_S2_State_t(RP1& _platform, ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel = nullptr) 
							: StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>(_platform, _controller, topLevel), controller{_controller},
							power(topLevel->power),
							s0_State{_controller, topLevel}, s1_State{_controller, topLevel}
							{};
						StateMachine_S2_State_t() = delete;
						~StateMachine_S2_State_t() = default;
					
					
					
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
							case s_s1:
							{
								s1_State.during();
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
									if((this->eT.get()) > (0))
									{
										s0_State.exit();
										s1_State.enter();
										currentState = s_s1;
										return true;	
									}
				
									break;
								}
								case s_s1:
								{
									{
										s1_State.exit();
										this-> = this->setPoint;
										currentState = j_j0;
										return true;	
									}
				
									break;
								}
								case j_i0:
								{
									
									;
									currentState = s_s1;
									s1_State.enter();
									return true;
								}
								case j_j0:
								{
									
									this->tryEmitInt_ActualHV(std::tuple<double>{0});;
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
				
				class S2_State_t : public CompositeState<StateMachine_S2_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>
				{
					public:
						S2_State_t(RP1& robot, ControllerType& controller, State_machine_StateMachine<ControllerType>* topLevel) : CompositeState<StateMachine_S2_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>(robot, controller, topLevel){}
						S2_State_t() = delete;
					
						void enter() override
						{
						
							CompositeState<StateMachine_S2_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::enter();
						}
					
						void exit() override
						{
				
							CompositeState<StateMachine_S2_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::exit();
						}
				
						void during() override
						{
				
							CompositeState<StateMachine_S2_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::during();
						}
				};
				class S3_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit S3_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						S3_State_t() = delete;
					
						void enter() override
						{
							this->controller.platform->checkLimits();
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				class S4_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit S4_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						S4_State_t() = delete;
					
						void enter() override
						{
							this->controller.platform->supplyVoltCheck();
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				S1_State_t s1_State;
				S2_State_t s2_State;
				S3_State_t s3_State;
				S4_State_t s4_State;
				
				enum PossibleStates
				{
					s_s1,
					s_s2,
					s_s3,
					s_s4,
					j_i0
				} currentState = j_i0; 
		
			public:
				StateMachine_ClosedLoop_State_t(RP1& _platform, ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel = nullptr) 
					: StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>(_platform, _controller, topLevel), controller{_controller},
					power(topLevel->power),
					s1_State{_controller, topLevel}, s2_State{_platform, _controller, topLevel}, s3_State{_controller, topLevel}, s4_State{_controller, topLevel}
					{};
				StateMachine_ClosedLoop_State_t() = delete;
				~StateMachine_ClosedLoop_State_t() = default;
			
			
			
				void execute() override
				{
					while(tryTransitions());
				
					switch(currentState)
					{
					case s_s1:
					{
						s1_State.during();
						break;
					}
					case s_s2:
					{
						s2_State.during();
						break;
					}
					case s_s3:
					{
						s3_State.during();
						break;
					}
					case s_s4:
					{
						s4_State.during();
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
						case s_s1:
						{
		
							break;
						}
						case s_s2:
						{
							if((this->lim) == (true))
							{
								s2_State.exit();
								s1_State.enter();
								currentState = s_s1;
								return true;	
							}
		
							break;
						}
						case s_s3:
						{
							if(((this->lim) == (false)) && ((this->eT.get()) > (0)))
							{
								s3_State.exit();
								s2_State.enter();
								currentState = s_s2;
								return true;	
							}
							if((this->lim) == (true))
							{
								s3_State.exit();
								s1_State.enter();
								currentState = s_s1;
								return true;	
							}
		
							break;
						}
						case s_s4:
						{
							if((this->lim) == (true))
							{
								s4_State.exit();
								s1_State.enter();
								currentState = s_s1;
								return true;	
							}
							if((this->lim) == (false))
							{
								s4_State.exit();
								this->tryEmitInt_pwmSignal(std::tuple<Power>{Power::On});
								s3_State.enter();
								currentState = s_s3;
								return true;	
							}
		
							break;
						}
						case j_i0:
						{
							
							;
							currentState = s_s4;
							s4_State.enter();
							return true;
						}
						default:
							break;
					}
						
					return false;
				}
		};
		
		class ClosedLoop_State_t : public CompositeState<StateMachine_ClosedLoop_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>
		{
			public:
				ClosedLoop_State_t(RP1& robot, ControllerType& controller, State_machine_StateMachine<ControllerType>* topLevel) : CompositeState<StateMachine_ClosedLoop_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>(robot, controller, topLevel){}
				ClosedLoop_State_t() = delete;
			
				void enter() override
				{
					this->sm.tryEmitCurrentState(std::tuple<State>{State::ClosedLoop});
				
					CompositeState<StateMachine_ClosedLoop_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::enter();
				}
			
				void exit() override
				{
		
					CompositeState<StateMachine_ClosedLoop_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::exit();
				}
		
				void during() override
				{
		
					CompositeState<StateMachine_ClosedLoop_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::during();
				}
		};
		class StateMachine_ErrorMode_State_t : public StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>
		{
			public:
				ControllerType& controller;
				Power& power;
			public:
				class F0_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit F0_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
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
				
				class S1_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit S1_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						S1_State_t() = delete;
					
						void enter() override
						{
							this->controller. = 0;
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				class S2_State_t : public robocalc::State<ControllerType>
				{
					public:
						explicit S2_State_t(ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel) 
							: robocalc::State<ControllerType>(_controller)
						{
						}
						
						S2_State_t() = delete;
					
						void enter() override
						{
							this->controller. = 0;
						}
						
						void exit() override
						{
						}
						
						void during() override
						{
						}
				};
				
				F0_State_t f0_State;
				S1_State_t s1_State;
				S2_State_t s2_State;
				
				enum PossibleStates
				{
					s_f0,
					s_s1,
					s_s2,
					j_i0,
					j_j0
				} currentState = j_i0; 
		
			public:
				StateMachine_ErrorMode_State_t(RP1& _platform, ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel = nullptr) 
					: StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>(_platform, _controller, topLevel), controller{_controller},
					power(topLevel->power),
					f0_State{_controller, topLevel}, s1_State{_controller, topLevel}, s2_State{_controller, topLevel}
					{};
				StateMachine_ErrorMode_State_t() = delete;
				~StateMachine_ErrorMode_State_t() = default;
			
			
			
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
					case s_s1:
					{
						s1_State.during();
						break;
					}
					case s_s2:
					{
						s2_State.during();
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
						case s_s1:
						{
							{
								s1_State.exit();
								this->tryEmitExt_setPoint(std::tuple<double>{this->setPoint});
								s2_State.enter();
								currentState = s_s2;
								return true;	
							}
		
							break;
						}
						case s_s2:
						{
							{
								s2_State.exit();
								this->tryEmitInt_ActualHV(std::tuple<double>{this->ActualHV});
								currentState = j_j0;
								return true;	
							}
		
							break;
						}
						case j_i0:
						{
							
							this-> = false;
							;
							currentState = s_s1;
							s1_State.enter();
							return true;
						}
						case j_j0:
						{
							if(
									((this->setPoint) == (0)) && ((this->ActualHV) == (0)))
							{
								this-> = true;
								;
								f0_State.enter();
								currentState = s_f0;
								return true;
							}
							if(
									((this->setPoint) != (0)) || ((this->ActualHV) != (0)))
							{
								;
								s1_State.enter();
								currentState = s_s1;
								return true;
							}
							
							break;
						}
						default:
							break;
					}
						
					return false;
				}
		};
		
		class ErrorMode_State_t : public CompositeState<StateMachine_ErrorMode_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>
		{
			public:
				ErrorMode_State_t(RP1& robot, ControllerType& controller, State_machine_StateMachine<ControllerType>* topLevel) : CompositeState<StateMachine_ErrorMode_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>(robot, controller, topLevel){}
				ErrorMode_State_t() = delete;
			
				void enter() override
				{
					this->sm.tryEmitCurrentState(std::tuple<State>{State::ErrorMode});
					this->controller.platform->disableHV(false);
				
					CompositeState<StateMachine_ErrorMode_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::enter();
				}
			
				void exit() override
				{
		
					CompositeState<StateMachine_ErrorMode_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::exit();
				}
		
				void during() override
				{
		
					CompositeState<StateMachine_ErrorMode_State_t, RP1, ControllerType, State_machine_StateMachine<ControllerType>>::during();
				}
		};
		Ramping_State_t ramping_State;
		Init_State_t init_State;
		Wait24Vpower_State_t wait24Vpower_State;
		ClosedLoop_State_t closedLoop_State;
		ErrorMode_State_t errorMode_State;
		
		enum PossibleStates
		{
			s_Ramping,
			s_Init,
			s_Wait24Vpower,
			s_ClosedLoop,
			s_ErrorMode,
			j_i0,
			j_j0
		} currentState = j_i0; 

	public:
		State_machine_StateMachine(RP1& _platform, ControllerType& _controller, State_machine_StateMachine<ControllerType>* topLevel = nullptr) 
			: StateMachine<RP1, ControllerType, State_machine_StateMachine<ControllerType>>(_platform, _controller, topLevel), controller{_controller},
			power(Power::On),
			ramping_State{_controller, this}, init_State{_platform, _controller, this}, wait24Vpower_State{_platform, _controller, this}, closedLoop_State{_platform, _controller, this}, errorMode_State{_platform, _controller, this}
			{};
		State_machine_StateMachine() = delete;
		~State_machine_StateMachine() = default;
	
	
		bool canReceiveInt_DisableHV(std::tuple<> args)
		{
			if(blockedByInt_DisableHV != nullptr)
				if(blockedByInt_DisableHV->getSender() == this)
					return false;
												
			blockedByInt_DisableHV = nullptr;
		
			switch(currentState)
			{
				case s_Ramping:
				{
					
					return false;
				}
				case s_Init:
				{
					
					return false;
				}
				case s_Wait24Vpower:
				{
					return true;
					
					return false;
				}
				case s_ClosedLoop:
				{
					return true;
					
					return false;
				}
				case s_ErrorMode:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByInt_DisableHV = nullptr;
															
		inline void tryEmitInt_DisableHV(std::tuple<> args)
		{
			blockedByInt_DisableHV = controller.channels.tryEmitInt_DisableHV(this, args);
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
		
		bool canReceiveExt_setPoint(std::tuple<double> args)
		{
			if(blockedByExt_setPoint != nullptr)
				if(blockedByExt_setPoint->getSender() == this)
					return false;
												
			blockedByExt_setPoint = nullptr;
		
			switch(currentState)
			{
				case s_Ramping:
				{
					
					return false;
				}
				case s_Init:
				{
					
					return false;
				}
				case s_Wait24Vpower:
				{
					
					return false;
				}
				case s_ClosedLoop:
				{
					
					return false;
				}
				case s_ErrorMode:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByExt_setPoint = nullptr;
															
		inline void tryEmitExt_setPoint(std::tuple<double> args)
		{
			blockedByExt_setPoint = controller.channels.tryEmitExt_setPoint(this, args);
		}
		
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
		
		bool canReceiveExt_pow24VStatus(std::tuple<Power> args)
		{
			if(blockedByExt_pow24VStatus != nullptr)
				if(blockedByExt_pow24VStatus->getSender() == this)
					return false;
												
			blockedByExt_pow24VStatus = nullptr;
		
			switch(currentState)
			{
				case s_Ramping:
				{
					
					return false;
				}
				case s_Init:
				{
					
					return false;
				}
				case s_Wait24Vpower:
				{
					
					return false;
				}
				case s_ClosedLoop:
				{
					
					return false;
				}
				case s_ErrorMode:
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
		
		bool canReceiveInt_overLimit(std::tuple<double> args)
		{
			if(blockedByInt_overLimit != nullptr)
				if(blockedByInt_overLimit->getSender() == this)
					return false;
												
			blockedByInt_overLimit = nullptr;
		
			switch(currentState)
			{
				case s_Ramping:
				{
					
					return false;
				}
				case s_Init:
				{
					
					return false;
				}
				case s_Wait24Vpower:
				{
					
					return false;
				}
				case s_ClosedLoop:
				{
					
					return false;
				}
				case s_ErrorMode:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByInt_overLimit = nullptr;
															
		inline void tryEmitInt_overLimit(std::tuple<double> args)
		{
			blockedByInt_overLimit = controller.channels.tryEmitInt_overLimit(this, args);
		}
		
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
		
		bool canReceiveInt_pwmSignal(std::tuple<Power> args)
		{
			if(blockedByInt_pwmSignal != nullptr)
				if(blockedByInt_pwmSignal->getSender() == this)
					return false;
												
			blockedByInt_pwmSignal = nullptr;
		
			switch(currentState)
			{
				case s_Ramping:
				{
					
					return false;
				}
				case s_Init:
				{
					
					return false;
				}
				case s_Wait24Vpower:
				{
					
					return false;
				}
				case s_ClosedLoop:
				{
					
					return false;
				}
				case s_ErrorMode:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByInt_pwmSignal = nullptr;
															
		inline void tryEmitInt_pwmSignal(std::tuple<Power> args)
		{
			blockedByInt_pwmSignal = controller.channels.tryEmitInt_pwmSignal(this, args);
		}
		
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
		
		bool canReceiveInt_underLimit(std::tuple<double> args)
		{
			if(blockedByInt_underLimit != nullptr)
				if(blockedByInt_underLimit->getSender() == this)
					return false;
												
			blockedByInt_underLimit = nullptr;
		
			switch(currentState)
			{
				case s_Ramping:
				{
					
					return false;
				}
				case s_Init:
				{
					
					return false;
				}
				case s_Wait24Vpower:
				{
					
					return false;
				}
				case s_ClosedLoop:
				{
					
					return false;
				}
				case s_ErrorMode:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByInt_underLimit = nullptr;
															
		inline void tryEmitInt_underLimit(std::tuple<double> args)
		{
			blockedByInt_underLimit = controller.channels.tryEmitInt_underLimit(this, args);
		}
		
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
		
		bool canReceiveInt_ActualHV(std::tuple<double> args)
		{
			if(blockedByInt_ActualHV != nullptr)
				if(blockedByInt_ActualHV->getSender() == this)
					return false;
												
			blockedByInt_ActualHV = nullptr;
		
			switch(currentState)
			{
				case s_Ramping:
				{
					
					return false;
				}
				case s_Init:
				{
					
					return false;
				}
				case s_Wait24Vpower:
				{
					
					return false;
				}
				case s_ClosedLoop:
				{
					
					return false;
				}
				case s_ErrorMode:
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
			case s_Ramping:
			{
				ramping_State.during();
				break;
			}
			case s_Init:
			{
				init_State.during();
				break;
			}
			case s_Wait24Vpower:
			{
				wait24Vpower_State.during();
				break;
			}
			case s_ClosedLoop:
			{
				closedLoop_State.during();
				break;
			}
			case s_ErrorMode:
			{
				errorMode_State.during();
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
				case s_Ramping:
				{
					if((this->eT.get()) >= (1))
					{
						ramping_State.exit();
						init_State.enter();
						currentState = s_Init;
						return true;	
					}

					break;
				}
				case s_Init:
				{
					{
						init_State.exit();
						wait24Vpower_State.enter();
						currentState = s_Wait24Vpower;
						return true;	
					}

					break;
				}
				case s_Wait24Vpower:
				{
					if((this->res) == (true))
					{
						wait24Vpower_State.exit();
						this-> = false;
						errorMode_State.enter();
						currentState = s_ErrorMode;
						return true;	
					}
					if((((this->setPoint) == (0)) && ((this->lim) == (false))) && ((this->res) == (false)))
					{
						wait24Vpower_State.exit();
						closedLoop_State.enter();
						currentState = s_ClosedLoop;
						return true;	
					}
					if(int_DisableHV_in.getSender() != nullptr)
					{
						wait24Vpower_State.exit();
						int_DisableHV_in.reset();
						currentState = j_j0;
						return true;	
					}

					break;
				}
				case s_ClosedLoop:
				{
					if((this->res) == (true))
					{
						closedLoop_State.exit();
						this-> = false;
						errorMode_State.enter();
						currentState = s_ErrorMode;
						return true;	
					}
					if(int_DisableHV_in.getSender() != nullptr)
					{
						closedLoop_State.exit();
						int_DisableHV_in.reset();
						currentState = j_j0;
						return true;	
					}

					break;
				}
				case s_ErrorMode:
				{
					if(this->errorAck)
					{
						errorMode_State.exit();
						wait24Vpower_State.enter();
						currentState = s_Wait24Vpower;
						return true;	
					}

					break;
				}
				case j_i0:
				{
					
					;
					currentState = s_Ramping;
					ramping_State.enter();
					return true;
				}
				case j_j0:
				{
					
					this->controller.platform->disableHV(true);
					this-> = false;
					;
					currentState = s_ErrorMode;
					errorMode_State.enter();
					return true;
				}
				default:
					break;
			}
				
			return false;
		}
};
	
#endif
