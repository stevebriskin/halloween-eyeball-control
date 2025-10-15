# Module halloween-eyes 

Code for the eye-follower halloween project

## Model stevebriskin:halloween-eyes:eyecontrol

Main control logic to track users.

### Configuration


#### Attributes

The following attributes are available for this model:

| Name                  | Type         | Inclusion | Description                                                    |
|-----------------------|--------------|-----------|----------------------------------------------------------------|
| `field_of_view`       | float        | Required  | Field of view angle of the camera          |
| `camera_name`         | string       | Required  | Name of the camera component to use for vision detection      |
| `vision_service_name` | string       | Required  | Name of the vision service to use for person detection        |
| `heads`               | []HeadConfig | Required  | Array of head configurations for multiple eye assemblies      |

#### Head Configuration

Each head in the `heads` array has the following attributes:

| Name                     | Type    | Inclusion | Description                                                |
|--------------------------|---------|-----------|-----------------------------------------------------------|
| `name`                   | string  | Required  | Name identifier for this head                             |
| `location_wrt_center`    | float   | Required  | Location relative to image center (0.5 = center)         |
| `board_name`             | string  | Optional  | Name of board component (required if light_pin provided)  |
| `light_pin`              | string  | Optional  | GPIO pin name for controlling light                       |
| `servo_eye_l`            | string  | Required  | Name of left eye servo component                          |
| `servo_eye_r`            | string  | Optional  | Name of right eye servo component                         |
| `invert_servo_direction` | bool    | Optional  | Whether to invert servo movement direction, default: false                |

#### Example Configuration

```
{
  "field_of_view" : 100,
  "camera_name": "camera-1",
  "vision_service_name": "vision-1",
  "heads": [
    {
      "name": "1",
      "location_wrt_center": 0.5,
      "board_name": "board-1",
      "light_pin": 37,
      "servo_eye_l": "eye-1-L",
      "servo_eye_r": "eye-1-R",
      "invert_servo_direction": true
    }
  ]
}
```





