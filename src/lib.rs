use cuvslam_lib::bindings;
use std::ffi::CString;

// Re-export key types
pub use cuvslam_lib::bindings::{
    CUVSLAM_Camera, CUVSLAM_CameraRig, CUVSLAM_Configuration, CUVSLAM_Image,
    CUVSLAM_Pose, CUVSLAM_PoseEstimate, CUVSLAM_Status, CUVSLAM_TrackerHandle,
};

/// Distortion model parameters for brown5k model (9 parameters)
pub struct Brown5kParameters {
    pub cx: f32,  // Principal point x
    pub cy: f32,  // Principal point y 
    pub fx: f32,  // Focal length x
    pub fy: f32,  // Focal length y
    pub k1: f32,  // Radial distortion coefficient 1
    pub k2: f32,  // Radial distortion coefficient 2
    pub k3: f32,  // Radial distortion coefficient 3
    pub p1: f32,  // Tangential distortion coefficient 1
    pub p2: f32,  // Tangential distortion coefficient 2
}

/// Distortion model parameters for pinhole model (4 parameters)
pub struct PinholeParameters {
    pub cx: f32,  // Principal point x
    pub cy: f32,  // Principal point y
    pub fx: f32,  // Focal length x
    pub fy: f32,  // Focal length y
}

/// Distortion model parameters for fisheye4 model (8 parameters)
pub struct Fisheye4Parameters {
    pub cx: f32,  // Principal point x
    pub cy: f32,  // Principal point y
    pub fx: f32,  // Focal length x
    pub fy: f32,  // Focal length y
    pub k1: f32,  // Fisheye distortion coefficient 1
    pub k2: f32,  // Fisheye distortion coefficient 2
    pub k3: f32,  // Fisheye distortion coefficient 3
    pub k4: f32,  // Fisheye distortion coefficient 4
}

/// Safe wrapper around camera parameters and configuration
pub struct Camera {
    parameters: Vec<f32>,
    distortion_model: CString,
    inner: CUVSLAM_Camera,
}

impl Camera {
    /// Create a new camera with brown5k distortion model
    pub fn new_brown5k(width: i32, height: i32, params: Brown5kParameters, pose: CUVSLAM_Pose) -> Self {
        let parameters = vec![
            params.cx, params.cy,
            params.fx, params.fy,
            params.k1, params.k2, params.k3,
            params.p1, params.p2
        ];
        let distortion_model = CString::new("brown5k").unwrap();
        
        let inner = CUVSLAM_Camera {
            width,
            height,
            distortion_model: distortion_model.as_ptr(),
            parameters: parameters.as_ptr(),
            num_parameters: 9,
            border_top: 0,
            border_bottom: 0,
            border_left: 0,
            border_right: 0,
            pose,
        };

        Self {
            parameters,
            distortion_model,
            inner,
        }
    }

    /// Create a new camera with pinhole model
    pub fn new_pinhole(width: i32, height: i32, params: PinholeParameters, pose: CUVSLAM_Pose) -> Self {
        let parameters = vec![
            params.cx, params.cy,
            params.fx, params.fy
        ];
        let distortion_model = CString::new("pinhole").unwrap();

        let inner = CUVSLAM_Camera {
            width,
            height,
            distortion_model: distortion_model.as_ptr(),
            parameters: parameters.as_ptr(),
            num_parameters: 4,
            border_top: 0,
            border_bottom: 0,
            border_left: 0,
            border_right: 0,
            pose,
        };

        Self {
            parameters,
            distortion_model,
            inner,
        }
    }

    /// Create a new camera with fisheye4 model
    pub fn new_fisheye4(width: i32, height: i32, params: Fisheye4Parameters, pose: CUVSLAM_Pose) -> Self {
        let parameters = vec![
            params.cx, params.cy,
            params.fx, params.fy,
            params.k1, params.k2,
            params.k3, params.k4
        ];
        let distortion_model = CString::new("fisheye4").unwrap();

        let inner = CUVSLAM_Camera {
            width,
            height,
            distortion_model: distortion_model.as_ptr(),
            parameters: parameters.as_ptr(),
            num_parameters: 8,
            border_top: 0,
            border_bottom: 0,
            border_left: 0,
            border_right: 0,
            pose,
        };

        Self {
            parameters,
            distortion_model,
            inner,
        }
    }

    /// Get a reference to the underlying CUVSLAM_Camera
    pub fn as_inner(&self) -> &CUVSLAM_Camera {
        &self.inner
    }
}

/// Safe wrapper around camera rig configuration
pub struct CameraRig {
    _inner_cameras: Vec<CUVSLAM_Camera>,
    _cameras: Vec<Camera>,
    inner: CUVSLAM_CameraRig,
}

impl CameraRig {
    /// Create a new camera rig from a vector of cameras
    pub fn new(cameras: Vec<Camera>) -> Self {
        let _inner_cameras: Vec<_> = cameras.iter().map(|c| c.inner.clone()).collect();
        let inner = CUVSLAM_CameraRig {
            cameras: _inner_cameras.as_ptr(),
            num_cameras: cameras.len() as i32,
        };

        Self { 
            _inner_cameras,  // Keep the cloned cameras alive
            _cameras: cameras,
            inner,
        }
    }

    /// Get a reference to the underlying CUVSLAM_CameraRig
    pub fn as_inner(&self) -> &CUVSLAM_CameraRig {
        &self.inner
    }
}

/// Status codes returned by CUVSLAM operations
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Status {
    /// Operation completed successfully
    Success,
    /// Tracking was lost
    TrackingLost,
    /// Invalid argument provided
    InvalidArg,
    /// Cannot localize in the current environment
    CannotLocalize,
    /// Generic/unknown error occurred
    GenericError,
    /// Unsupported number of cameras
    UnsupportedNumberOfCameras,
    /// SLAM is not initialized
    SlamNotInitialized,
    /// Operation is not implemented
    NotImplemented,
    /// Reading SLAM internals is disabled
    ReadingSlamInternalsDisabled,
}

impl From<cuvslam_lib::bindings::CUVSLAM_Status> for Status {
    fn from(status: cuvslam_lib::bindings::CUVSLAM_Status) -> Self {
        match status {
            cuvslam_lib::bindings::CUVSLAM_SUCCESS => Status::Success,
            cuvslam_lib::bindings::CUVSLAM_TRACKING_LOST => Status::TrackingLost,
            cuvslam_lib::bindings::CUVSLAM_INVALID_ARG => Status::InvalidArg,
            cuvslam_lib::bindings::CUVSLAM_CAN_NOT_LOCALIZE => Status::CannotLocalize,
            cuvslam_lib::bindings::CUVSLAM_GENERIC_ERROR => Status::GenericError,
            cuvslam_lib::bindings::CUVSLAM_UNSUPPORTED_NUMBER_OF_CAMERAS => Status::UnsupportedNumberOfCameras,
            cuvslam_lib::bindings::CUVSLAM_SLAM_IS_NOT_INITIALIZED => Status::SlamNotInitialized,
            cuvslam_lib::bindings::CUVSLAM_NOT_IMPLEMENTED => Status::NotImplemented,
            cuvslam_lib::bindings::CUVSLAM_READING_SLAM_INTERNALS_DISABLED => Status::ReadingSlamInternalsDisabled,
            _ => Status::GenericError,
        }
    }
}

impl std::fmt::Display for Status {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Status::Success => write!(f, "Success"),
            Status::TrackingLost => write!(f, "Tracking Lost"),
            Status::InvalidArg => write!(f, "Invalid Argument"),
            Status::CannotLocalize => write!(f, "Cannot Localize"),
            Status::GenericError => write!(f, "Generic Error"),
            Status::UnsupportedNumberOfCameras => write!(f, "Unsupported Number of Cameras"),
            Status::SlamNotInitialized => write!(f, "SLAM Not Initialized"),
            Status::NotImplemented => write!(f, "Not Implemented"),
            Status::ReadingSlamInternalsDisabled => write!(f, "Reading SLAM Internals Disabled"),
        }
    }
}

/// Safe wrapper around CUVSLAM tracker
pub struct Tracker {
    handle: CUVSLAM_TrackerHandle,
    _rig: CameraRig, // Keep rig alive while tracker exists
}

impl Tracker {
    /// Create a new tracker instance
    pub fn new(rig: CameraRig, config: &CUVSLAM_Configuration) -> Result<Self, Status> {
        let mut handle = std::ptr::null_mut();
        
        unsafe {
            let status = bindings::CUVSLAM_CreateTracker(&mut handle, rig.as_inner(), config);
            if status == 0 {
                Ok(Self { handle, _rig: rig })
            } else {
                Err(status.into())
            }
        }
    }

    /// Track current frame synchronously
    pub fn track(
        &self,
        images: &[CUVSLAM_Image],
        predicted_pose: Option<&PoseEstimate>,
    ) -> Result<PoseEstimate, Status> {
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
                predicted_pose.map_or(std::ptr::null(), |p| &p.pose),
                &mut pose_estimate,
            );

            if status == 0 {
                Ok(pose_estimate.into())
            } else {
                Err(status.into())
            }
        }
    }

    /// Get current odometry pose
    pub fn get_odometry_pose(&self) -> Result<CUVSLAM_Pose, Status> {
        let mut pose = CUVSLAM_Pose {
            r: [0.0; 9],
            t: [0.0; 3],
        };

        unsafe {
            let status = bindings::CUVSLAM_GetOdometryPose(self.handle, &mut pose);
            if status == 0 {
                Ok(pose)
            } else {
                Err(status.into())
            }
        }
    }

    /// Save SLAM database to folder
    pub fn save_to_slam_db(&self, folder: &str) -> Result<(), Status> {
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
                Err(status.into())
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

/// Image encoding formats supported by the tracker
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImageEncoding {
    /// 8-bit monochrome image
    Mono8,
    /// 8-bit RGB image 
    Rgb8,
}

impl From<cuvslam_lib::bindings::CUVSLAM_ImageEncoding> for ImageEncoding {
    fn from(encoding: cuvslam_lib::bindings::CUVSLAM_ImageEncoding) -> Self {
        match encoding {
            cuvslam_lib::bindings::CUVSLAM_ImageEncoding_MONO8 => ImageEncoding::Mono8,
            cuvslam_lib::bindings::CUVSLAM_ImageEncoding_RGB8 => ImageEncoding::Rgb8,
            _ => panic!("Unknown image encoding"),
        }
    }
}

/// A pose estimate with timestamp and covariance information
#[derive(Debug, Clone)]
pub struct PoseEstimate {
    /// The estimated pose
    pub pose: CUVSLAM_Pose,
    /// Timestamp in nanoseconds
    pub timestamp_ns: i64,
    /// 6x6 covariance matrix in row-major format
    /// The parameters are: (rotation_x, rotation_y, rotation_z, x, y, z)
    /// Rotations are in radians, translations in meters
    pub covariance: [f32; 36],
}

impl From<PoseEstimate> for CUVSLAM_PoseEstimate {
    fn from(est: PoseEstimate) -> Self {
        CUVSLAM_PoseEstimate {
            pose: est.pose,
            timestamp_ns: est.timestamp_ns,
            covariance: est.covariance,
        }
    }
}

impl From<CUVSLAM_PoseEstimate> for PoseEstimate {
    fn from(est: CUVSLAM_PoseEstimate) -> Self {
        PoseEstimate {
            pose: est.pose,
            timestamp_ns: est.timestamp_ns,
            covariance: est.covariance,
        }
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
        
        // Create left camera
        let left_cam = Camera::new_brown5k(
            640, 480,
            Brown5kParameters {
                cx: 320.0, cy: 240.0,
                fx: 500.0, fy: 500.0,
                k1: 0.0, k2: 0.0, k3: 0.0,
                p1: 0.0, p2: 0.0
            },
            CUVSLAM_Pose {
                r: [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0],
                t: [0.0, 0.0, 0.0],
            }
        );

        // Create right camera
        let right_cam = Camera::new_brown5k(
            640, 480,
            Brown5kParameters {
                cx: 320.0, cy: 240.0,
                fx: 500.0, fy: 500.0,
                k1: 0.0, k2: 0.0, k3: 0.0,
                p1: 0.0, p2: 0.0
            },
            CUVSLAM_Pose {
                r: [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0],
                t: [0.1, 0.0, 0.0],
            }
        );

        let rig = CameraRig::new(vec![left_cam, right_cam]);
        let tracker = Tracker::new(rig, &config);
        
        match &tracker {
            Ok(_) => println!("Tracker initialized successfully"),
            Err(status) => println!("Failed to initialize tracker with status: {}", status),
        }
        assert!(tracker.is_ok());
    }
}
