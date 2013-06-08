# Sensor Abstraction Layer #

Many small embedded systems exist to collect data from sensors, analyse the data, and either take an appropriate action or send that sensor data to another system for processing.

One of the many challenges of embedded systems design is the fact that parts you used today may be out of production tommorow, or system requirements may change and you may need to choose a different sensor down the road.

Creating new drivers is a relatively easy task, but integrating them into existing systems is both error prone and time consuming since sensors rarely use the exact same units of measurement.

By reducing all data to a single **sensors\_event\_t** 'type' and settling on specific, **standardised SI units** for each sensor family the same sensor types return values that are comparable with any other similar sensor.  This enables you to switch sensor models with very little impact on the rest of the system, which can help mitigate some of the risks and problems of sensor availability and code reuse.

The sensor abstraction layer is also useful for data-logging and data-transmission.  The abstraction layer provides a single, well-understood and fixed-length record that can be recorded in your log files or transmitted to a target device without needing to know any HW-specific information.  Helper functions to that effect are provided in sensors.c, such as serialising sensor events and sensor details for transmission over the air or over a wire.

## How Does it Work? ##

There are two main typedefs and one enum defined in sensors.h that must be used by any sensor driver that you want to be compliant with the abstraction layer:

**Sensor Types (sensors\_type\_t)**

These pre-defined sensor types are used to properly handle the two related typedefs below, and allows us determine what types of units the sensor uses, etc.

```
/** Sensor types */
typedef enum
{
  SENSOR_TYPE_ACCELEROMETER         = (1),
  SENSOR_TYPE_MAGNETIC_FIELD        = (2),
  SENSOR_TYPE_ORIENTATION           = (3),
  SENSOR_TYPE_GYROSCOPE             = (4),
  SENSOR_TYPE_LIGHT                 = (5),
  SENSOR_TYPE_PRESSURE              = (6),
  SENSOR_TYPE_PROXIMITY             = (8),
  SENSOR_TYPE_GRAVITY               = (9),
  SENSOR_TYPE_LINEAR_ACCELERATION   = (10),
  SENSOR_TYPE_ROTATION_VECTOR       = (11),
  SENSOR_TYPE_RELATIVE_HUMIDITY     = (12),
  SENSOR_TYPE_AMBIENT_TEMPERATURE   = (13),
  SENSOR_TYPE_VOLTAGE               = (15),
  SENSOR_TYPE_CURRENT               = (16)
} sensors_type_t;
```

**Sensor Details (sensor\_t)**

This typedef describes the specific capabilities of this sensor, and allows us to know what sensor we are using beneath the abstraction layer.

```
/* Sensor details (40 bytes) */
/** struct sensor_s is used to describe basic information about a specific sensor. */
typedef struct
{
    char     name[12];
    int32_t  version;
    int32_t  sensor_id;
    int32_t  type;
    float    max_value;
    float    min_value;
    float    resolution;
    int32_t  min_delay;
} sensor_t;
```

The individual fields are intended to be used as follows:

- **name**: The sensor name or ID, up to a maximum of twelve characters (ex. "MPL115A2")
- **version**: The version of the sensor HW and the driver to allow us to differentiate versions of the board or driver
- **sensor\_id**: A unique sensor identifier that is used to differentiate this specific sensor instance from any others that are present on the system or in the sensor network
- **type**: The sensor type, based on **sensors\_type\_t** in sensors.h
- **max\_value**: The maximum value that this sensor can return (in the appropriate SI unit)
- **min\_value**: The minimum value that this sensor can return (in the appropriate SI unit)
- **resolution**: The smallest difference between two values that this sensor can report (in the appropriate SI unit)
- **min\_delay**: The minimum delay in microsecond between two sensor event, or '0' if there is no constant sensor rate

**Sensor Data/Events (sensors\_event\_t)**

This typedef is used to return sensor data from any sensor supported by the abstraction layer, using standard SI units and scales.

```
/* Sensor event (36 bytes) */
/** struct sensor_event_s is used to provide a single sensor event in a common format. */
typedef struct
{
    int32_t version;
    int32_t sensor_id;
    int32_t type;
    int32_t reserved0;
    int32_t timestamp;
    union
    {
        float           data[4];
        sensors_vec_t   acceleration;
        sensors_vec_t   magnetic;
        sensors_vec_t   orientation;
        sensors_vec_t   gyro;
        float           temperature;
        float           distance;
        float           light;
        float           pressure;
        float           relative_humidity;
        float           current;
        float           voltage;
    };
} sensors_event_t;
```
It includes the following fields:

- **version**: Contain 'sizeof(sensors\_event\_t)' to identify which version of the API we're using in case this changes in the future
- **sensor\_id**: A unique sensor identifier that is used to differentiate this specific sensor instance from any others that are present on the system or in the sensor network (must match the sensor\_id value in the corresponding sensor\_t enum above!)
- **type**: the sensor type, based on **sensors\_type\_t** in sensors.h
- **timestamp**: time in milliseconds when the sensor value was read
- **data[4]**: An array of four 32-bit values that allows us to encapsulate any type of sensor data via a simple union (further described below)

**Standardised SI values for sensors\_event\_t**

A key part of the abstraction layer is the standardisation of values on SI units of a particular scale, which is accomplished via the data[4] union in sensors\_event\_t above.  This 16 byte union includes fields for each main sensor type, and uses the following SI units and scales:

- **acceleration**: values are in **meter per second per second** (m/s^2)
- **magnetic**: values are in **micro-Tesla** (uT)
- **orientation**: values are in **degrees**
- **gyro**: values are in **rad/s**
- **temperature**: values in **degrees centigrade** (Celsius) 
- **distance**: values are in **centimeters**
- **light**: values are in **SI lux** units
- **pressure**: values are in **hectopascal** (hPa)
- **relative\_humidity**: values are in **percent**
- **current**: values are in **milliamps** (mA)
- **voltage**: values are in **volts** (V)

## Adding sensors.c Support to an Existing Sensor Driver ##

Sensor drivers should still be written in the traditional way, using efficient fixed-point math and the native units of the sensor.  

The sensor abstraction layer should sit on top of these efficient, light-weight sensor drivers, and can be made to work with any existing sensor driver by simply adding two new functions on top of it:

**Function One: Get Sensor Details**

You need to create a function that populates a **sensor\_t** object, and that accurately describes the capabilities of this sensor.  An example of how to do this (for the l3gd20 gyroscope) is shown below:

```
/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void l3gd20GetSensor(sensor_t *sensor)
{
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "L3GD20", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _l3gd20SensorID;
  sensor->type        = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay   = 0;

  /* We need to do some calculations to determine resolution and maxRange in rad/s */
  sensor->max_value   = _l3gd20MeasurementRange * SENSORS_DPS_TO_RADS;
  sensor->min_value   = 0;
  switch (_l3gd20MeasurementRange)
  {
    case 2000:
      sensor->resolution  = (_l3gd20MeasurementRange / 32767.0F) * L3GD20_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
      break;
    case 500:
      sensor->resolution  = (_l3gd20MeasurementRange / 32767.0F) * L3GD20_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS;
      break;
    case 250:
    default:
      sensor->resolution  = (_l3gd20MeasurementRange / 32767.0F) * L3GD20_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
      break;
  }
}
```

**Function Two: Get Sensor Event**

The second function that is required must populate a **sensors\_event\_t** object, containing the sensor event data with the values converted to the appropriate SI units and scale.

Another example using the l3gd20 gyroscope can be seen below, where the raw DPS (degrees per second) values provided by the sensor are converted to the rad/s units defined by the sensor abstraction layer, including compensation for the gain settings on the gyroscope:

```
/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
error_t l3gd20GetSensorEvent(sensors_event_t *event)
{
  l3gd20Data_t data;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _l3gd20SensorID;
  event->type      = SENSOR_TYPE_GYROSCOPE;
  event->timestamp = delayGetTicks();

  /* Retrieve values from the sensor */
  ASSERT_STATUS(l3gd20Poll(&data));

  /* The L3GD20 returns degrees per second, adjusted by sensitivity which
   * is shown as mdps/digit in the datasheet.  To convert this to proper
   * degrees/s multiply by the appropriate sensitivity value and then
   * convert it to the rad/s value that sensors_event_t is expecting. */
  switch (_l3gd20MeasurementRange)
  {
    case (2000):
      event->gyro.x = data.x * L3GD20_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
      event->gyro.y = data.y * L3GD20_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
      event->gyro.z = data.z * L3GD20_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
      break;
    case (500):
      event->gyro.x = data.x * L3GD20_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS;
      event->gyro.y = data.y * L3GD20_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS;
      event->gyro.z = data.z * L3GD20_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS;
      break;
    case (250):
    default:
      event->gyro.x = data.x * L3GD20_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
      event->gyro.y = data.y * L3GD20_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
      event->gyro.z = data.z * L3GD20_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
      break;
  }

  return ERROR_NONE;
}
```

## The Abstraction Layer in Practice ##

Using the sensor abstraction layer is relatively easy once a compliant driver has been created.  

Every compliant sensor can now be read using a single, well-known 'type' (sensors\_event\_t), and there is a standardised way of interrogating a sensor about its specific capabilities (via sensor\_t).

An example of reading the l3gd20 gyroscope above can be seen below:

```
error_t error;
sensors_event_t event;

error = l3gd20GetSensorEvent(&event);
if (!error)
{
  /* Check the X value */
  if (event.gyro.x > 0.5F)
  {
    // Do something
  }
}
```

Similarly, we can get the basic technical capabilities of this sensor with the following code:

```
sensor_t sensor;

// Get the sensor_t data
l3gd20GetSensor(&sensor);

// Display the resolution of this sensor
printf("Gyroscope: %s %s", sensor.name, CFG_PRINTF_NEWLINE);
printf("Resolution: %f rad/s %s", sensor.resolution, CFG_PRINTF_NEWLINE); 

```
