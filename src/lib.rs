use cuvslam_lib::bindings;
use std::ffi::CString;

// Re-export key types
pub use cuvslam_lib::bindings::{
    CUVSLAM_Camera, CUVSLAM_CameraRig, CUVSLAM_Configuration, CUVSLAM_Image,
    CUVSLAM_Pose, CUVSLAM_PoseEstimate, CUVSLAM_Status, CUVSLAM_TrackerHandle,
    CUVSLAM_ImageEncoding_MONO8, CUVSLAM_ImageEncoding_RGB8,
};

/// Safe wrapper around CUVSLAM tracker
pub struct Tracker {
    handle: CUVSLAM_TrackerHandle,
}

impl Tracker {
    /// Create a new tracker instance
    pub fn new(rig: &CUVSLAM_CameraRig, config: &CUVSLAM_Configuration) -> Result<Self, CUVSLAM_Status> {
        let mut handle = std::ptr::null_mut();
        
        unsafe {
            let status = bindings::CUVSLAM_CreateTracker(&mut handle, rig, config);
            if status == 0 {
                Ok(Self { handle })
            } else {
                Err(status)
            }
        }
    }

    /// Track current frame synchronously
    pub fn track(
        &self,
        images: &[CUVSLAM_Image],
        predicted_pose: Option<&CUVSLAM_Pose>,
    ) -> Result<CUVSLAM_PoseEstimate, CUVSLAM_Status> {
        let mut pose_estimate = CUVSLAM_PoseEstimate {
            pose: CUVSLAM_Pose {
                r: [0.0; 9],
                t: [0.0; 3],
            },
            timestamp_ns: 0,
            covariance: [0.0; 36],
        };

        unsafe {
            let status = bindings::CUVSLAM_Track(
                self.handle,
                images.as_ptr(),
                images.len(),
                predicted_pose.map_or(std::ptr::null(), |p| p),
                &mut pose_estimate,
            );

            if status == 0 {
                Ok(pose_estimate)
            } else {
                Err(status)
            }
        }
    }

    /// Get current odometry pose
    pub fn get_odometry_pose(&self) -> Result<CUVSLAM_Pose, CUVSLAM_Status> {
        let mut pose = CUVSLAM_Pose {
            r: [0.0; 9],
            t: [0.0; 3],
        };

        unsafe {
            let status = bindings::CUVSLAM_GetOdometryPose(self.handle, &mut pose);
            if status == 0 {
                Ok(pose)
            } else {
                Err(status)
            }
        }
    }

    /// Save SLAM database to folder
    pub fn save_to_slam_db(&self, folder: &str) -> Result<(), CUVSLAM_Status> {
        let folder = CString::new(folder).unwrap();
        unsafe {
            let status = bindings::CUVSLAM_SaveToSlamDb(
                self.handle,
                folder.as_ptr(),
                None,
                std::ptr::null_mut(),
            );
            if status == 0 {
                Ok(())
            } else {
                Err(status)
            }
        }
    }
}

impl Drop for Tracker {
    fn drop(&mut self) {
        unsafe {
            bindings::CUVSLAM_DestroyTracker(self.handle);
        }
    }
}

/// Initialize default CUVSLAM configuration
pub fn init_default_configuration() -> CUVSLAM_Configuration {
    unsafe { bindings::CUVSLAM_GetDefaultConfiguration() }
}

/// Get CUVSLAM version information
pub fn get_version() -> (i32, i32, Option<String>) {
    let mut major = 0;
    let mut minor = 0;
    let mut version_ptr = std::ptr::null();

    unsafe {
        bindings::CUVSLAM_GetVersion(&mut major, &mut minor, &mut version_ptr);
        
        let version = if !version_ptr.is_null() {
            // Convert C string to Rust String
            let c_str = std::ffi::CStr::from_ptr(version_ptr);
            Some(c_str.to_string_lossy().into_owned())
        } else {
            None
        };

        (major, minor, version)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        let (major, minor, version) = get_version();
        println!("Version info - major: {}, minor: {}, version: {:?}", major, minor, version);
        assert!(major >= 0);
        assert!(minor >= 0);
        assert!(version.is_some());
    }

    #[test]
    fn test_tracker_initialization() {
        let config = init_default_configuration();
        
        // Create parameters arrays for brown5k distortion model
        // Parameters: cx, cy, fx, fy, k1, k2, k3, p1, p2
        let params1 = [320.0f32, 240.0, 500.0, 500.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let params2 = [320.0f32, 240.0, 500.0, 500.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        
        // Use brown5k distortion model as specified in header
        let dist_model = CString::new("brown5k").unwrap();
        
        // Create camera array with second camera offset to the right by 10cm
        let cameras = [
            CUVSLAM_Camera {
                width: 640,
                height: 480,
                distortion_model: dist_model.as_ptr(),
                parameters: params1.as_ptr(),
                num_parameters: 9,
                border_top: 0,
                border_bottom: 0, // 0 means use full frame
                border_left: 0,
                border_right: 0,
                pose: CUVSLAM_Pose {
                    r: [1.0, 0.0, 0.0,  // Identity rotation matrix
                        0.0, 1.0, 0.0,  // stored in column-major order
                        0.0, 0.0, 1.0], 
                    t: [0.0, 0.0, 0.0], // No translation for first camera
                },
            },
            CUVSLAM_Camera {
                width: 640,
                height: 480,
                distortion_model: dist_model.as_ptr(),
                parameters: params2.as_ptr(),
                num_parameters: 9,
                border_top: 0,
                border_bottom: 0,
                border_left: 0,
                border_right: 0,
                pose: CUVSLAM_Pose {
                    r: [1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0],
                    t: [0.1, 0.0, 0.0], // 10cm offset to the right
                },
            },
        ];

        let rig = CUVSLAM_CameraRig {
            num_cameras: 2,
            cameras: cameras.as_ptr(),
        };

        let tracker = Tracker::new(&rig, &config);
        match &tracker {
            Ok(_) => println!("Tracker initialized successfully"),
            Err(status) => println!("Failed to initialize tracker with status: {}", status),
        }
        assert!(tracker.is_ok());
    }
}
