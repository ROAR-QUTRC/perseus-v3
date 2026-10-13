export interface StampedHeaderType {
  stamp?: object; // filled by rosbridge
  frame_id?: string;
}

// std_msgs types
export interface Float64MultiArrayType {
  data: Array<number>;
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

// Service types
export interface EmptyRequestType {}

export interface EmptyResponseType {}

export interface SetBoolRequestType {
  data: boolean;
}

export interface SetBoolResponseType {
  success: boolean;
  message: string;
}
