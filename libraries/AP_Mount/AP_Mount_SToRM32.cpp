#include "AP_Mount_SToRM32.h"
#if HAL_MOUNT_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Mount_SToRM32::AP_Mount_SToRM32(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _chan(MAVLINK_COMM_0)
{}

// update mount position - should be called periodically
void AP_Mount_SToRM32::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT:
            {
            const Vector3f &target = _state._retract_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            stabilize();
            resend_now = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            stabilize();
            resend_now = true;
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true)) {
                stabilize();
                resend_now = true;
            }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (calc_angle_to_sysid_target(_angle_ef_target_rad, true, true)) {
                stabilize();
                resend_now = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // resend target angles at least once per second
    if (resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_RESEND_MS)) {
        send_do_mount_control(_angle_bf_output_deg.y, _angle_bf_output_deg.x, _angle_bf_output_deg.z, MAV_MOUNT_MODE_MAVLINK_TARGETING);
    }
}

// stabilize - stabilizes the mount relative to the Earth's frame
//  input: _angle_ef_target_rad (earth frame targets in radians)
//  output: _angle_bf_output_deg (body frame angles in degrees)
void AP_Mount_SToRM32::stabilize()
{
    AP_AHRS &ahrs = AP::ahrs();
    // only do the full 3D frame transform if we are doing pan control
    if (_state._stab_pan) {
        Matrix3f m;                         ///< holds 3 x 3 matrix, var is used as temp in calcs
        Matrix3f cam;                       ///< Rotation matrix earth to camera. Desired camera from input.
        Matrix3f gimbal_target;             ///< Rotation matrix from plane to camera. Then Euler angles to the servos.
        m = ahrs.get_rotation_body_to_ned();
        m.transpose();
        cam.from_euler(_angle_ef_target_rad.x, _angle_ef_target_rad.y, _angle_ef_target_rad.z);
        gimbal_target = m * cam;
        gimbal_target.to_euler(&_angle_bf_output_deg.x, &_angle_bf_output_deg.y, &_angle_bf_output_deg.z);
        _angle_bf_output_deg.x  = _state._stab_roll ? degrees(_angle_bf_output_deg.x) : degrees(_angle_ef_target_rad.x);
        _angle_bf_output_deg.y  = _state._stab_tilt ? degrees(_angle_bf_output_deg.y) : degrees(_angle_ef_target_rad.y);
        _angle_bf_output_deg.z = degrees(_angle_bf_output_deg.z);
    } else {
        // otherwise base mount roll and tilt on the ahrs
        // roll/tilt attitude, plus any requested angle
        _angle_bf_output_deg.x = degrees(_angle_ef_target_rad.x);
        _angle_bf_output_deg.y = degrees(_angle_ef_target_rad.y);
        _angle_bf_output_deg.z = degrees(_angle_ef_target_rad.z);
        if (_state._stab_roll) {
            _angle_bf_output_deg.x -= degrees(ahrs.roll);
        }
        if (_state._stab_tilt) {
            _angle_bf_output_deg.y -= degrees(ahrs.pitch);
        }

        // lead filter
        const Vector3f &gyro = ahrs.get_gyro();

        if (_state._stab_roll && !is_zero(_state._roll_stb_lead) && fabsf(ahrs.pitch) < M_PI/3.0f) {
            // Compute rate of change of euler roll angle
            float roll_rate = gyro.x + (ahrs.sin_pitch() / ahrs.cos_pitch()) * (gyro.y * ahrs.sin_roll() + gyro.z * ahrs.cos_roll());
            _angle_bf_output_deg.x -= degrees(roll_rate) * _state._roll_stb_lead;
        }

        if (_state._stab_tilt && !is_zero(_state._pitch_stb_lead)) {
            // Compute rate of change of euler pitch angle
            float pitch_rate = ahrs.cos_pitch() * gyro.y - ahrs.sin_roll() * gyro.z;
            _angle_bf_output_deg.y -= degrees(pitch_rate) * _state._pitch_stb_lead;
        }
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_SToRM32::has_pan_control() const
{
    // we do not have yaw control
    return false;
}

// set_mode - sets mount's mode
void AP_Mount_SToRM32::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_SToRM32::send_mount_status(mavlink_channel_t chan)
{
    // return target angles as gimbal's actual attitude.  To-Do: retrieve actual gimbal attitude and send these instead
    mavlink_msg_mount_status_send(chan, 0, 0, ToDeg(_angle_ef_target_rad.y)*100, ToDeg(_angle_ef_target_rad.x)*100, ToDeg(_angle_ef_target_rad.z)*100);
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_SToRM32::find_gimbal()
{
    // return immediately if initialised
    if (_initialised) {
        return;
    }

    // return if search time has has passed
    if (AP_HAL::millis() > AP_MOUNT_STORM32_SEARCH_MS) {
        return;
    }

    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GIMBAL, _sysid, _compid, _chan)) {
        _initialised = true;
    }
}

// send_do_mount_control - send a COMMAND_LONG containing a do_mount_control message
void AP_Mount_SToRM32::send_do_mount_control(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // reverse pitch and yaw control
    pitch_deg = -pitch_deg;
    yaw_deg = -yaw_deg;

    // send command_long command containing a do_mount_control command
    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_MOUNT_CONTROL,
                                  0,        // confirmation of zero means this is the first time this message has been sent
                                  pitch_deg,
                                  roll_deg,
                                  yaw_deg,
                                  0, 0, 0,  // param4 ~ param6 unused
                                  mount_mode);

    // store time of send
    _last_send = AP_HAL::millis();
}
#endif // HAL_MOUNT_ENABLED
