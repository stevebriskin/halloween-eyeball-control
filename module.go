package halloweeneyeballcontrol

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	"go.viam.com/rdk/components/board"
	"go.viam.com/rdk/components/camera"
	generic "go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/components/servo"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/utils"
)

var (
	EyeControl       = resource.NewModel("stevebriskin", "halloween-eyes", "eye-control")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterComponent(generic.API, EyeControl,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newHalloweenEyeballControlEyecontrol,
		},
	)
}

type Head struct {
	name                   string
	locationWrtImageCenter float64
	lightPin               board.GPIOPin
	servoEyeL              servo.Servo
	servoEyeR              servo.Servo
	invertServoDirection   bool
}

type HeadConfig struct {
	name                   string  `json:"name"`
	locationWrtImageCenter float64 `json:"location_wrt_center"`
	boardName              string  `json:"board_name"`
	lightPin               string  `json:"light_pin"`
	servoEyeL              string  `json:"servo_eye_l"`
	servoEyeR              string  `json:"servo_eye_r"`
	invertServoDirection   bool    `json:"invert_servo_direction"`
}

type Config struct {
	fieldOfView            float64      `json:"field_of_view"`
	cameraName             string       `json:"camera_name"`
	visionServiceName      string       `json:"vision_service_name"`
	heads                  []HeadConfig `json:"heads"`
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit required (first return) and optional (second return) dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	var dependencies []string

	logging.NewLogger("validate").Infof("Validating config: %+v", cfg)

	// camera is required
	if cfg.cameraName == "" {
		return nil, nil, fmt.Errorf("camera_name is required")
	}
	if cfg.visionServiceName == "" {
		return nil, nil, fmt.Errorf("vision_service_name is required")
	}

	dependencies = append(dependencies, cfg.cameraName)
	dependencies = append(dependencies, cfg.visionServiceName)

	for _, head := range cfg.heads {
		if head.name == "" {
			return nil, nil, fmt.Errorf("head name is required")
		}
		if head.locationWrtImageCenter == 0 {
			return nil, nil, fmt.Errorf("location_wrt_center is required")
		}

		if head.lightPin != "" {
			if head.boardName == "" {
				return nil, nil, fmt.Errorf("board_name is required when light_pin is provided")
			}

			dependencies = append(dependencies, head.boardName)
		}

		// left is required
		if head.servoEyeL == "" {
			return nil, nil, fmt.Errorf("servo_eye_l is required")
		}
		dependencies = append(dependencies, head.servoEyeL)

		// right is optional
		if head.servoEyeR != "" {
			dependencies = append(dependencies, head.servoEyeR)
		}

	}

	if len(cfg.heads) == 0 {
		return nil, nil, fmt.Errorf("at least one head is required")
	}

	return dependencies, nil, nil
}

type halloweenEyeballControlEyecontrol struct {
	resource.AlwaysRebuild
	name resource.Name

	logger logging.Logger
	cfg    *Config

	camera        camera.Camera
	visionService vision.Service
	heads         []Head

	cancelCtx  context.Context
	cancelFunc func()
}

func newHalloweenEyeballControlEyecontrol(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewEyeControl(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func newHead(headConfig HeadConfig, deps resource.Dependencies, logger logging.Logger) (*Head, error) {
	var lightPin board.GPIOPin
	if headConfig.lightPin != "" {
		b, err := board.FromDependencies(deps, headConfig.boardName)
		if err != nil {
			logger.Error(err)
			return nil, err
		}

		lightPin, err = b.GPIOPinByName(headConfig.lightPin)
		if err != nil {
			logger.Error(err)
			return nil, err
		}
	}

	// left is required
	servoEyeL, err := resource.FromDependencies[servo.Servo](deps, servo.Named(headConfig.servoEyeL))
	if err != nil {
		logger.Error(err)
		return nil, err
	}

	var servoEyeR servo.Servo
	if headConfig.servoEyeR != "" {
		servoEyeR, err = resource.FromDependencies[servo.Servo](deps, servo.Named(headConfig.servoEyeR))
		if err != nil {
			logger.Error(err)
			return nil, err
		}
	}

	return &Head{
		name:                   headConfig.name,
		locationWrtImageCenter: headConfig.locationWrtImageCenter,
		lightPin:               lightPin,
		servoEyeL:              servoEyeL,
		servoEyeR:              servoEyeR,
		invertServoDirection:   headConfig.invertServoDirection,
	}, nil
}

func NewEyeControl(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	camera, err := camera.FromDependencies(deps, conf.cameraName)
	if err != nil {
		cancelFunc()
		return nil, err
	}

	visionService, err := vision.FromDependencies(deps, conf.visionServiceName)
	if err != nil {
		cancelFunc()
		return nil, err
	}

	heads := make([]Head, len(conf.heads))
	for i, headConf := range conf.heads {
		head, err := newHead(headConf, deps, logger)
		if err != nil {
			cancelFunc()
			return nil, err
		}
		heads[i] = *head
	}
	s := &halloweenEyeballControlEyecontrol{
		name:          name,
		logger:        logger,
		cfg:           conf,
		camera:        camera,
		visionService: visionService,
		heads:         heads,
		cancelCtx:     cancelCtx,
		cancelFunc:    cancelFunc,
	}
	return s, nil
}

func (s *halloweenEyeballControlEyecontrol) Name() resource.Name {
	return s.name
}

func (s *halloweenEyeballControlEyecontrol) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *halloweenEyeballControlEyecontrol) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}

func (s *halloweenEyeballControlEyecontrol) Run() error {
	imageWidth, _, err := getImageSize(s.cancelCtx, s.camera, s.logger)
	if err != nil {
		s.logger.Error(err)
		return err
	}
	s.logger.Infof("Image width: %d", imageWidth)

	for {
		select {
		case <-s.cancelCtx.Done():
			return s.cancelCtx.Err()
		default:
		}

		lowestPersonDetectionCenter, err := getLowestPersonDetections(s.cancelCtx, s.visionService, s.logger)
		if err != nil {
			s.logger.Error(err)
			continue
		}

		s.logger.Infof("Lowest person detection center: %d", lowestPersonDetectionCenter)

		var wg sync.WaitGroup
		for _, head := range s.heads {
			wg.Add(1)
			go func(h Head) {
				defer wg.Done()
				err := h.process(s.cancelCtx, lowestPersonDetectionCenter, imageWidth, s.cfg.fieldOfView, s.logger)
				if err != nil {
					s.logger.Error(err)
				}
			}(head)
		}
		wg.Wait()
		time.Sleep(20 * time.Microsecond)

	}
}

func (head *Head) process(ctx context.Context, lowestPersonDetectionCenter int, imageWidth int, fieldOfView float64, logger logging.Logger) error {
	// no person detected, move to default position
	if lowestPersonDetectionCenter < 0 {
		err := head.moveServoToAngle(context.Background(), 90, logger)
		if err != nil {
			return err
		}
		if head.lightPin != nil {
			head.lightPin.Set(context.Background(), false, nil)
		}

	} else {
		percentOfImageWidth := float64(lowestPersonDetectionCenter) / float64(imageWidth)

		currentAngle, err := head.servoEyeL.Position(context.Background(), nil)
		if err != nil {
			return err
		}

		targetAngle := head.computeTargetAngle(percentOfImageWidth, fieldOfView)
		logger.Infof("%s: Percent of image width: %.1f, Target angle: %.1f degrees", head.name, percentOfImageWidth, targetAngle)

		if head.lightPin != nil {
			head.lightPin.Set(context.Background(), true, nil)
		}

		// intentionally overshoot a bit since there's inherent lag. 90 is special since it's the rest position.
		if currentAngle != 90 {
			targetAngle += (targetAngle - float64(currentAngle)) / 2.0
		}

		// check if the angle is valid
		err = head.moveServoToAngle(context.Background(), targetAngle, logger)
		if err != nil {
			return err
		}
	}
	return nil
}

func (head *Head) moveServoToAngle(ctx context.Context, angle float64, logger logging.Logger) error {
	logger.Debugf("%s: Moving servos to angle: %f", head.name, angle)
	_, err := utils.RunInParallel(ctx, []utils.SimpleFunc{
		func(ctx context.Context) error {
			return head.servoEyeL.Move(ctx, uint32(angle), nil)
		},
		func(ctx context.Context) error {
			if head.servoEyeR != nil {
				return head.servoEyeR.Move(ctx, uint32(angle), nil)
			}
			return nil
		},
	})

	if err != nil {
		logger.Error("%s: Error moving servos to angle: %f, err: %v", head.name, angle, err)
		return err
	}

	logger.Debugf("%s: Servos moved to angle: %f", head.name, angle)
	return nil
}

// TODO do some trigonometry
func (head *Head) computeTargetAngle(percentOfImageWidth float64, fieldOfView float64) float64 {
	// Calculate the difference between target location and detected person location
	difference := head.locationWrtImageCenter - percentOfImageWidth

	var targetAngle float64
	if difference == 0 {
		// Person is at center, servo should be at 90 degrees
		targetAngle = 90
	} else if difference < 0 {
		// Person is to the right of center, map to 90-180 degrees proportionately
		// difference ranges from -0.5 to 0, map to 90 to 180
		targetAngle = 90 + (-difference * fieldOfView)
	} else {
		// Person is to the left of center, map to 0-90 degrees proportionately
		// difference ranges from 0 to 0.5, map to 90 to 0
		targetAngle = 90 - (difference * fieldOfView)
	}

	if head.invertServoDirection {
		// Invert the angle around 90 degrees (90 - (angle - 90) = 180 - angle)
		targetAngle = 180 - targetAngle
	}

	return targetAngle
}

func getLowestPersonDetections(ctx context.Context, vision1Service vision.Service, logger logging.Logger) (int, error) {
	// Get detections from vision service
	detections, err := vision1Service.DetectionsFromCamera(ctx, "camera-1", nil)
	if err != nil {
		logger.Error(err)
		return -1, err
	}

	logger.Infof("Received %d detections from vision service", len(detections))

	var lowestPersonDetectionCenter int = -1
	// Get the lowest # coordinate of a Person detection
	for i, detection := range detections {
		logger.Debugf("Detection %d:", i)
		logger.Debugf("  Class Name: %s", detection.Label())
		logger.Debugf("  Confidence: %.2f", detection.Score())
		if detection.BoundingBox() != nil {
			bbox := *detection.BoundingBox()
			logger.Debugf("  Bounding Box: Min(%.2f, %.2f) Max(%.2f, %.2f)",
				bbox.Min.X, bbox.Min.Y, bbox.Max.X, bbox.Max.Y)
		}

		if detection.Label() == "Person" {
			bbox := *detection.BoundingBox()
			width := bbox.Max.X - bbox.Min.X
			centerOfDetection := bbox.Min.X + width/2
			if centerOfDetection > lowestPersonDetectionCenter {
				lowestPersonDetectionCenter = centerOfDetection
			}
		}
	}

	return lowestPersonDetectionCenter, nil
}

func getImageSize(ctx context.Context, camera camera.Camera, logger logging.Logger) (int, int, error) {
	images, _, err := camera.Images(ctx, nil, nil)
	if err != nil {
		logger.Error(err)
		return 0, 0, err
	}
	if len(images) == 0 {
		logger.Error("No images found")
		return 0, 0, errors.New("no images found")
	}
	image, err := images[0].Image(ctx)
	if err != nil {
		logger.Error(err)
		return 0, 0, err
	}
	return image.Bounds().Max.X, image.Bounds().Max.Y, nil
}
