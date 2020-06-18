#JSON Configuration Parameters

## Software
Anything to do software related.

- **frames**: The number of frames to capture. Integer.
- **dummy_frames**: The number of frames to capture before capturing [May enhance performance]. Integer.
- **filters**: The filters to apply to the each frame. A list of lists - each sublist contains the name of the filter and its
    arguments (if needed).
- **after_filters**: The filters to apply to final resulting frame, after applying the previous filters to each frame.
    Same structure as frames.  
    The names for them in the json file are: _'decimation_filter'_, _'threshold_filter'_, _'disparity_transform'_,
    _'spatial_filter'_, _'temporal_filter'_, _'hole_filling_filter'_  
    You can find the filter parameters here: https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md  
- **filename**: The output file name (With extension).

## Cameras
Camera related settings.

- **serial**: The serials number of the camera. String.
- **angle**: The angle to rotate the points generated from that camera. Float.
- **distance**: The distance of the camera from the middle point. Float.

## Hardware
Something related to hardware.  
Maybe add fov restrictions, or something like that.  
Not sure about this segment (Unused for now).  