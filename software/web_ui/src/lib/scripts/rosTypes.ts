export interface StampedHeaderType {
  stamp?: object; // filled by rosbridge
  frame_id?: string;
}

// std_msgs types
export interface Float64MultiArrayType {
  data: Array<number>;
}

export interface BoolType {
  data: boolean;
}

export interface TwistType {
  linear: {
    x: number;
    y: number;
    z: number;
  };
  angular: {
    x: number;
    y: number;
    z: number;
  };
}

export interface TwistStampedType {
  header: StampedHeaderType;
  twist: TwistType;
}

// actuator_msgs types
export interface ActuatorsMessageType {
  header: StampedHeaderType;
  position: Array<number>; // float64
  velocity: Array<number>; // float64
  normalized: Array<number>; // float64
}

// sensor_msgs types
export interface JointStateType {
  header: StampedHeaderType;
  name: Array<string>;
  position: Array<number>; // float64
  velocity: Array<number>; // float64
  effort: Array<number>; // float64
}

// nav_msgs types
export interface OdometryType {
		header: StampedHeaderType;
		pose: {
			pose: {
				position: { x: number; y: number; z: number };
				orientation: { x: number; y: number; z: number; w: number };
			};
			covariance: number[];
		};
		twist: {
			twist: {
				linear: { x: number; y: number; z: number };
				angular: { x: number; y: number; z: number };
			};
			covariance: number[];
		};
	}

// Service types
export interface EmptyRequestType {}

export interface EmptyResponseType {}

export interface TriggerType {
  request: EmptyRequestType;
  response: {
    success: boolean;
    message: string;
  };
}

export interface SetBoolRequestType {
  data: boolean;
}

export interface SetBoolResponseType {
  success: boolean;
  message: string;
}
