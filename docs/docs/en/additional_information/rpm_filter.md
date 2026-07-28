# RPM Filter

## What Is an RPM Filter?

---

An RPM filter is a notch filter centered on the motor's rotational frequency and its harmonics.
In drones, especially multicopters, most vibration affecting the IMU is caused by the propellers.
By making the notch filter's center frequency track the motor speed, this vibration can be effectively removed.

## Effects of the RPM Filter

---

The greatest advantage of an RPM filter is its extremely high filtering effectiveness.
The gain at the center frequency of a notch filter is zero, so vibration at that frequency is ideally eliminated completely.
In practice, center-frequency and discretization errors prevent vibration from being eliminated completely,
but the filtering is still far more effective than with a low-pass filter.

The images below show IMU time-series data and frequency analysis results recorded while flying an F450.
Gray represents the raw data, and blue represents the filtered data.
The left column shows results with only a 40 Hz cutoff LPF applied, while the right column shows results with only the RPM filter applied.
Although the LPF suppresses high frequencies, strong vibration at the motor's rotational frequency remains.
In contrast, the RPM filter also significantly suppresses vibration at the motor's rotational frequency and its second and third harmonics.

<img src="../../../assets/rpm_filter/lpf_imu.png" alt="lpf_imu" width="49%"/>
<img src="../../../assets/rpm_filter/notch_imu.png" alt="notch_imu" width="49%"/>

<img src="../../../assets/rpm_filter/lpf_imu_fft.png" alt="lpf_imu_fft" width="49%"/>
<img src="../../../assets/rpm_filter/notch_imu_fft.png" alt="notch_imu_fft" width="49%"/>

Reducing IMU vibration, particularly gyroscope vibration, also reduces fluctuations in motor speed.
In the images below, blue represents the measured RPM and red represents the target RPM. The RPM fluctuates less when the notch filter is used.
This not only improves attitude-control stability by preventing unwanted forces,
but also reduces motor heat and produces a smoother propeller sound.

<img src="../../../assets/rpm_filter/lpf_rpm.png" alt="lpf_rpm" width="49%"/>
<img src="../../../assets/rpm_filter/notch_rpm.png" alt="notch_rpm" width="49%"/>

## Supported FMUs

---

The RPM filter is available on the following FMUs.

- FC2xx

## Parameters

---

![param_tuning](../../assets/rpm_filter/param_tuning.png)

Filter settings can be configured under `IMU Filter` in `Param Tuning` on the GCS.
The following sections describe each parameter, its default value, and how to tune it.

### `lowpass_filter/*_cutoff`

These are the cutoff frequencies of low-pass filters separate from the RPM filter.
Initially, set all three to 0 Hz. This disables the filters.
Because the RPM filter cannot suppress vibration from sources other than the motors and propellers, selectively enable the LPFs if strong vibration remains.

### `rpm_filter/quality_factor`

This value determines the sharpness of the notch filter.
Higher values make the filter sharper (narrower bandwidth) while reducing delay.
The default value is 0, which disables the filter.
Initially, set it to 5 for a good balance between delay and bandwidth.
After a test flight, if the FFT confirms that vibration has been suppressed enough that no peak is visible,
increase the value one step at a time until just before a peak begins to remain.

### `rpm_filter/min_center_frequency`, `rpm_filter/fade_range`

These parameters define the range of center frequencies over which the RPM filter is applied.
If the former is $f_\mathrm{m}$ and the latter is $f_\mathrm{f}$, the notch filter is applied to the center frequency $f_\mathrm{c}$ as follows.

- $f_\mathrm{c} \lt f_\mathrm{m}$: Filter disabled
- $f_\mathrm{m} \le f_\mathrm{c} \lt f_\mathrm{m} + f_\mathrm{f}$: Transition region (the filter becomes stronger as $f_\mathrm{c}$ increases)
- $f_\mathrm{m} + f_\mathrm{f} \le f_\mathrm{c}$: Filter enabled

The disabled range is provided because filtering low frequencies near the control bandwidth may adversely affect responsiveness and stability.
The transition region prevents abrupt changes in the overall filter characteristics when the RPM filter switches between enabled and disabled.
For a hovering rotational frequency of $f_\mathrm{h}$,
initially set approximately $f_\mathrm{m}=\frac{f_\mathrm{h}}{2}$ and $f_\mathrm{f}=\frac{f_\mathrm{h}}{4}$.

### `rpm_filter/lpf_cutoff`

This is the cutoff frequency of the low-pass filter applied to the RPM filter's center frequency.
The motor speed obtained from the ESC fluctuates considerably. Using it directly as the center frequency would cause excessive variation in the filter characteristics, so it is smoothed.
In most cases, the default value is appropriate.
