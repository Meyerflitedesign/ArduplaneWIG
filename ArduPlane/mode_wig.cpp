#include "mode.h"
#include "Plane.h"

bool ModeWIG::_enter()
{
    // Check if the vehicle has a downward facing rangefinder
    if (!plane.rangefinder.has_orientation(ROTATION_PITCH_270)) {
        plane.gcs().send_text(MAV_SEVERITY_ERROR, "Cannot enter WIG mode: No downward facing rangefinder");
        return false;
    }

    // Check if rangefinder is healthy
    if (plane.rangefinder.status_orient(ROTATION_PITCH_270) != RangeFinder::Status::Good) {
        plane.gcs().send_text(MAV_SEVERITY_ERROR, "Cannot enter WIG mode: Rangefinder not healthy");
        return false;
    }

    // Get altitude reading from rangefinder
    float altitude_ground_m = plane.rangefinder.distance_orient(ROTATION_PITCH_270);
    
    // Only enter if altitude is below WIG_ALT_MAX and above WIG_ALT_MIN
    if (altitude_ground_m < plane.g2.wig_alt_min_cm * 0.01f ||
        altitude_ground_m > plane.g2.wig_alt_max_cm * 0.01f) {
        plane.gcs().send_text(MAV_SEVERITY_ERROR, "Cannot enter WIG mode: Altitude out of range");
        return false;
    }

    plane.gcs().send_text(MAV_SEVERITY_ERROR, "Entering WIG mode");
    return true;
}

void ModeWIG::update()
{
    // set nav_roll using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();

    // Ignore pitch input from sticks, instead use rangefinder to maintain altitude
    float altitude_ground_m = plane.rangefinder.distance_orient(ROTATION_PITCH_270);
    float altitude_error_m = plane.g2.wig_alt_des_cm * 0.01f - altitude_ground_m;

    // Simple proportional control
    plane.nav_pitch_cd = constrain_int32(altitude_error_m*10000, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max*100);

    // Write to gcs
    // plane.gcs().send_text(MAV_SEVERITY_INFO, "nav_pitch=%.2f", plane.nav_pitch_cd * 0.01f);

    // Not needed?
    //plane.adjust_nav_pitch_throttle();
    
    // Should never happen with WIG?
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}

void ModeWIG::run()
{
    // Run base class function and then output throttle
    Mode::run();

    output_pilot_throttle();
}
