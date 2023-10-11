#include "Copter.h"
#define MODE_STAB 0
#define MODE_LOIT 1

#define ON_GROUND        0
#define IN_AIR_LOITER    1
uint8_t last_mode=0;
uint16_t test_loop_count=0;
uint8_t in_air_flag=0;
uint8_t ground_loit_manual=0;//log for two status when takeoff
uint8_t log_loop=0;
static uint32_t ground_detector_count=0;
static uint32_t air_detector_count = 0;//2023-09-23

bool ModeStab_to_Loit::init(bool ignore_checks)
{
   in_air_flag=false;
   loiter_nav->init_target();
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
   if(g2.user_parameters.stab_loit_alt < 20) 
   {
		g2.user_parameters.stab_loit_alt.set_and_save(20);
   }
   	
   copter.pos_control->set_ground_air_status(ON_GROUND);
   return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStab_to_Loit::run()
{   
	float target_roll, target_pitch;
	float target_climb_rate = 0.0f;
	uint8_t judge_status=0;//2023-09-08
    	get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);
    	loiter_nav->set_pilot_desired_acceleration(target_roll,target_pitch);
    	float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    	target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
        test_loop_count++;
	 uint32_t temp_time_elapse=0;
	if(copter.stab_to_loit_time >0)
	{
	   temp_time_elapse=AP_HAL::millis()-copter.stab_to_loit_time;
	  // if(target_climb_rate>0 || temp_time_elapse>4000)2023-08-27
	   if( temp_time_elapse>((uint32_t)g2.user_parameters.speed_recovery_time))
	   {
	   	
		copter.stab_to_loit_time=0;
		//g2.pilot_speed_dn.set(150);
		//g.pilot_speed_up.set(200);
		g2.pilot_speed_dn.load();
		g.pilot_speed_up.load();
		gcs().send_text(MAV_SEVERITY_CRITICAL, "set to origin:%d",(int)g2.pilot_speed_dn); 
	   }
	}
	if(ON_GROUND == in_air_flag)
	{ 	 
	    ground_detector_count=0;//2023-09-20
		pos_control->relax_z_controller(0.0f); 
    		if (!motors->armed()) 
   		{
       		 motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    		} 
   		else 
   		{
        		motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    		}

   		switch (motors->get_spool_state())      // Motors Stopped
    		{
    			case AP_Motors::SpoolState::SHUT_DOWN:
       		 	attitude_control->reset_yaw_target_and_rate();
        			attitude_control->reset_rate_controller_I_terms();
        			break;

    			case AP_Motors::SpoolState::GROUND_IDLE:     // Landed
       		 	attitude_control->reset_yaw_target_and_rate();
        			attitude_control->reset_rate_controller_I_terms_smoothly();
        			break;

    			case AP_Motors::SpoolState::THROTTLE_UNLIMITED:      // clear landing flag above zero throttle
       			if (!motors->limit.throttle_lower) 
        			{
            				set_land_complete(false);
        			}
       		 	break;

   			 case AP_Motors::SpoolState::SPOOLING_UP:
    		 	 case AP_Motors::SpoolState::SPOOLING_DOWN:   // do nothing
       			 break;
   		 }

		//no input loiter
		if((fabs(target_roll)<500 && fabs(target_pitch)<500) && copter.position_ok())//if(0) //(ekf_has_absolute_position() || ekf_has_relative_position());
		{
			loiter_nav->update();
			//2023-09-11
			//attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, loiter_nav->get_pitch(), target_yaw_rate);//set roll zero
			attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw((float)ahrs.roll_sensor*0.01, loiter_nav->get_pitch(), target_yaw_rate);//set roll zero
			ground_loit_manual=1;
			if(test_loop_count>600)
   	 		{
        			test_loop_count=0;
       			gcs().send_text(MAV_SEVERITY_CRITICAL, "ground loit:%0.1f",loiter_nav->get_pitch());
    			}
		}
		else //pf have pilot-input  stablize
		{
			loiter_nav->init_target();//
			attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    		ground_loit_manual=0;
			if(test_loop_count>400)
   	 		{
        			test_loop_count=0;
       			//gcs().send_text(MAV_SEVERITY_CRITICAL, "ground stab :%0.1f t_pit:%0.1f",target_roll,target_pitch);
    		}
		}
       	attitude_control->set_throttle_out(get_pilot_desired_throttle(),true,g.throttle_filt);
       	//2023-08-26  loit control or stablize??? 
       	float temp_rangerfinder_data=(float)copter.rangefinder_state.alt_cm_filt.get();
    	bool climb_rate_high = (int16_t)inertial_nav.get_velocity_z_up_cms() > (int16_t)(g2.user_parameters.stab2loit_up);//up speed high
     	bool rngfinder_hight_ok=(int16_t)temp_rangerfinder_data > (int16_t)(g2.user_parameters.stab_loit_alt);

      
    	float throttle = motors->get_throttle();
    	float half_hover_thr = throttle_hover()*0.2;//when bat low not work ok 09-24
    	bool thr_ok=throttle>half_hover_thr;//motors->get_throttle();


        //log status 
		judge_status=0;
		if(climb_rate_high)judge_status=judge_status|0x01;
		else judge_status=judge_status&0xFE;
     		
		if(rngfinder_hight_ok)judge_status=judge_status|0x02;
		else judge_status=judge_status&0xFD;

		if(copter.ap.throttle_zero) judge_status=judge_status|0x04;
		else judge_status=judge_status&0xFB;

		if(thr_ok) judge_status=judge_status|0x08;
		else judge_status=judge_status&0xF7;


    	if(copter.position_ok() && !copter.ap.throttle_zero && rngfinder_hight_ok && climb_rate_high && thr_ok)
    	{

    			copter.stab_to_loit_time=AP_HAL::millis();//2023-6-21 record change time
    			air_detector_count=0;
    			//2023-09-18
    			//loiter_nav->init_target();
    			//pos_control->relax_z_controller(0.0f); 
			if (!pos_control->is_active_z()) 
   			{
        			pos_control->init_z_controller();
    			}
    			in_air_flag=IN_AIR_LOITER;///BUG!!!!
    			copter.pos_control->set_ground_air_status(IN_AIR_LOITER);
    			gcs().send_text(MAV_SEVERITY_CRITICAL, "mode stab to loiter:%0.1f",temp_rangerfinder_data);
    			//gcs().send_text(MAV_SEVERITY_CRITICAL, "set to 10:%d %0.1f",(int)g2.pilot_speed_dn); 		
    	}
	}
	
	else //if(IN_AIR_LOITER == in_air_flag)//LOITER MODE
	{
		motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        loiter_nav->update();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
        copter.surface_tracking.update_surface_offset();
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        pos_control->update_z_controller();
        	
        bool descent_rate_low = ((int16_t)fabsf(inertial_nav.get_velocity_z_up_cms()) < (int16_t)(g2.user_parameters.loit2stab_down));//down speed low ,xi
		float temp_rangerfinder_data=copter.rangefinder_state.alt_cm_filt.get();
		bool rng_data_ok=((int16_t)temp_rangerfinder_data) < (int16_t)g2.user_parameters.loit_to_stab_alt;
		if(rng_data_ok) ground_detector_count++;
		else ground_detector_count=0;

//log judge ststus
		judge_status=0;
		if(descent_rate_low)judge_status=judge_status|0x01;
		else judge_status=judge_status&0xFE;

     	if(rng_data_ok)judge_status=judge_status|0x02;
		else judge_status=judge_status&0xFD;

		if(copter.ap.throttle_zero) judge_status=judge_status|0x04;
		else judge_status=judge_status&0xFB;
		
		if (copter.ap.throttle_zero && rng_data_ok && descent_rate_low) 
		{
			 ground_detector_count=0;
			 in_air_flag=ON_GROUND;
			 copter.pos_control->set_ground_air_status(ON_GROUND);
			 ground_detector_count=0;//2023-09-20
    			gcs().send_text(MAV_SEVERITY_CRITICAL, "mode loiter to stab"); 
		}
		
		if(test_loop_count>600)
   	 	{
        		test_loop_count=0;
       		//gcs().send_text(MAV_SEVERITY_CRITICAL, "loit:%d %d %0.1f",copter.ap.throttle_zero,descent_rate_low,temp_rangerfinder_data);
    	}
     }
    log_loop++;
    if(log_loop>20)
    {
        log_loop=0;
       	 copter.Log_Write_Stab_to_loit_msg(motors->get_throttle(),(int16_t)copter.rangefinder_state.alt_cm_filt.get(),(int16_t)inertial_nav.get_velocity_z_up_cms(),in_air_flag,ground_loit_manual,judge_status);
    }
        //2023-07-15 
}


void ModeStab_to_Loit::exit()
{
    copter.pos_control->set_ground_air_status(IN_AIR_LOITER);
    g2.pilot_speed_dn.load();
    g.pilot_speed_up.load();
   // on_ground_times=0;//2023-09-20
    gcs().send_text(MAV_SEVERITY_CRITICAL, "exit set to origin:%d",(int)g2.pilot_speed_dn); 
}
   
