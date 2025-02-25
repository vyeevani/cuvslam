use cuvslam::{
    Brown5kParameters, Camera, CameraRig, PoseEstimate, Status, Tracker,
    CUVSLAM_Configuration, CUVSLAM_Image, CUVSLAM_Pose,
};
use realsense_rust::{
    config::Config,
    context,
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind, Rs2Option},
    pipeline,
    frame,
};
use std::{hash::Hash, time::{Duration, SystemTime, UNIX_EPOCH}};
use std::collections::HashSet;
use rerun::{self, LoggableBatch};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize Rerun for visualization
    let rec = rerun::RecordingStreamBuilder::new("CUVSLAM RealSense Tracker").spawn()?;
    
    // Initialize RealSense
    let ctx = context::Context::new()?;
    let pipeline = pipeline::InactivePipeline::try_from(&ctx)?;

    // Print all active streams in the context
    for device in ctx.query_devices(HashSet::new()) {
        println!("Device: {}", device.info(Rs2CameraInfo::Name).map(|s| s.to_string_lossy()).unwrap_or_default());
        println!("Number of sensors: {}", device.sensors().len());
        for mut sensor in device.sensors() {
            println!("  Sensor: {}", sensor.info(Rs2CameraInfo::Name).map(|s| s.to_string_lossy()).unwrap_or_default());
            if let Err(e) = sensor.set_option(Rs2Option::EmitterEnabled, 0.) {
                println!("Unable to set option EmitterOnOff: {}", e);
            } else {
                println!("Successfully set option EmitterOnOff");
            }
        }
    }
    
    // Configure RealSense pipeline
    let mut config = Config::new();
    config.enable_stream(
        Rs2StreamKind::Infrared, 
        Some(1), // Left IR camera
        640, 
        480, 
        Rs2Format::Y8,
        30,
    )?;
    config.enable_stream(
        Rs2StreamKind::Infrared,
        Some(2), // Right IR camera
        640,
        480,
        Rs2Format::Y8,
        30,
    )?;

    // Start the pipeline
    let mut active_pipeline = pipeline.start(Some(config))?;

    // Create SLAM configuration
    let slam_config = cuvslam::init_default_configuration();

    // Create stereo camera rig
    let camera_rig = create_stereo_camera_rig();
    
    // Initialize SLAM tracker
    let tracker = match Tracker::new(camera_rig, &slam_config) {
        Ok(tracker) => tracker,
        Err(status) => {
            eprintln!("Failed to initialize tracker: {}", status);
            return Ok(());
        }
    };

    println!("Starting SLAM tracking...");
            
    // Main loop
    loop {
        // Wait for next frame
        let frames = active_pipeline.wait(Some(Duration::from_millis(10000)))?;
        
        // Get color frames using the CompositeFrame utility
        let infrared_frames: Vec<frame::InfraredFrame> = frames.frames_of_type();

        if infrared_frames.len() < 2 {
            eprintln!("Not enough color frames received!");
            continue;
        }

        // Convert to CUVSLAM images
        let mut images = vec![
            create_cuvslam_image(&infrared_frames[0]),
            create_cuvslam_image(&infrared_frames[1]),
        ];

        images[0].camera_index = 0;
        images[1].camera_index = 1;

        // Track frame
        match tracker.track(&images, None) {
            Ok(pose_estimate) => {
                print_pose(&pose_estimate);
                
                // Log pose to Rerun
                let t = &pose_estimate.pose.t;
                let r = &pose_estimate.pose.r;

                // Convert raw pointer to slice for Rerun
                let width = images[0].width as usize;
                let height = images[0].height as usize;
                let image_data = unsafe { 
                    std::slice::from_raw_parts(images[0].pixels, width * height)
                };
                
                rec.log("camera_image", &rerun::Image::new(image_data, rerun::ImageFormat::from_color_model([640, 480], rerun::ColorModel::L, rerun::ChannelDatatype::U8)))?;
                
                rec.log("camera_translation", &rerun::Transform3D::from_translation(rerun::Vec3D::new(t[0], t[1], t[2])))?;             
                /*
                rec.log(
                    "camera", 
                    &rerun::Transform3D::from_translation_rotation(
                        rerun::Vec3D::new(t[0], t[1], t[2]), 
                        rerun::Rotation3D::Quaternion(rerun::Quaternion::from_xyzw([r[0], r[1], r[2], r[3]]).into())
                    )
                )?;
                */
            }
            Err(Status::TrackingLost) => {
                println!("Tracking lost!");
            }
            Err(status) => {
                eprintln!("Error during tracking: {}", status);
                break;
            }
        }
    }

    Ok(())
}

fn create_stereo_camera_rig() -> CameraRig {
    // Create left camera (values are examples - replace with actual calibration)
    let left_cam = Camera::new_brown5k(
        640, 480,
        Brown5kParameters {
            cx: 320.0, cy: 240.0,
            fx: 385.0, fy: 385.0,
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

    // Create right camera with baseline offset
    let right_cam = Camera::new_brown5k(
        640, 480,
        Brown5kParameters {
            cx: 320.0, cy: 240.0,
            fx: 385.0, fy: 385.0,
            k1: 0.0, k2: 0.0, k3: 0.0,
            p1: 0.0, p2: 0.0
        },
        CUVSLAM_Pose {
            r: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0],
            t: [0.055, 0.0, 0.0], // 55mm baseline - adjust for your camera
        }
    );

    CameraRig::new(vec![left_cam, right_cam])
}

fn create_cuvslam_image(frame: &frame::InfraredFrame) -> CUVSLAM_Image {
    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as i64;

    CUVSLAM_Image {
        width: frame.width() as i32,
        height: frame.height() as i32,
        pitch: frame.stride() as i32,
        pixels: unsafe { frame.get_data() as *const _ as *const u8 },
        camera_index: 0, // Set appropriate camera index
        timestamp_ns: timestamp,
        image_encoding: 0, // Set appropriate encoding
    }
}

fn print_pose(pose_estimate: &PoseEstimate) {
    let t = &pose_estimate.pose.t;
    println!(
        "Position: x={:.3}, y={:.3}, z={:.3} meters",
        t[0], t[1], t[2]
    );
} 