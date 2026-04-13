"""
ADS1293 register map and conversion utilities.

Ported from CleverHand-interface:
  clvHd_module_ADS1293EMG_registers.hpp
  clvHd_module_ADS1293EMG.hpp
"""

# =============================================================================
# Register addresses
# =============================================================================
CONFIG_REG          = 0x00
FLEX_CH0_CN_REG     = 0x01
FLEX_CH1_CN_REG     = 0x02
FLEX_CH2_CN_REG     = 0x03
FLEX_PACE_CN_REG    = 0x04
FLEX_VBAT_CN_REG    = 0x05
LOD_CN_REG          = 0x06
LOD_EN_REG          = 0x07
LOD_CURRENT_REG     = 0x08
LOD_AC_CN_REG       = 0x09
CMDET_EN_REG        = 0x0A
CMDET_CN_REG        = 0x0B
RLD_CN_REG          = 0x0C
WILSON_EN1_REG      = 0x0D
WILSON_EN2_REG      = 0x0E
WILSON_EN3_REG      = 0x0F
WILSON_CN_REG       = 0x10
REF_CN_REG          = 0x11
OSC_CN_REG          = 0x12
AFE_RES_REG         = 0x13
AFE_SHDN_CN_REG     = 0x14
AFE_FAULT_CN_REG    = 0x15
AFE_PACE_CN_REG     = 0x17
ERROR_LOD_REG       = 0x18
ERROR_STATUS_REG    = 0x19
ERROR_RANGE1_REG    = 0x1A
ERROR_RANGE2_REG    = 0x1B
ERROR_RANGE3_REG    = 0x1C
ERROR_SYNC_REG      = 0x1D
ERROR_MISC_REG      = 0x1E
DIGO_STRENGTH_REG   = 0x1F
R2_RATE_REG         = 0x21
R3_RATE_CH0_REG     = 0x22
R3_RATE_CH1_REG     = 0x23
R3_RATE_CH2_REG     = 0x24
R1_RATE_REG         = 0x25
DIS_EFILTER_REG     = 0x26
DRDYB_SRC_REG       = 0x27
SYNCB_CN_REG        = 0x28
MASK_DRDYB_REG      = 0x29
MASK_ERR_REG        = 0x2A
RESERVED_0x2D_REG   = 0x2D
ALARM_FILTER_REG    = 0x2E
CH_CNFG_REG         = 0x2F
DATA_STATUS_REG     = 0x30
DATA_CH0_PACE_REG   = 0x31
DATA_CH1_PACE_REG   = 0x33
DATA_CH2_PACE_REG   = 0x35
DATA_CH0_ECG_REG    = 0x37
DATA_CH1_ECG_REG    = 0x3A
DATA_CH2_ECG_REG    = 0x3D
REVID_REG           = 0x40

# Operating modes
MODE_START_CONV  = 0x01
MODE_STANDBY     = 0x02
MODE_POWER_DOWN  = 0x04

# =============================================================================
# ADC max values (depend on R2/R3 filter settings)
# =============================================================================

def compute_adc_max(R2: int, R3: int) -> tuple[int, int]:
    """Compute (fast_adc_max, precise_adc_max) for given R2 and R3.

    Ported from clvHd_module_ADS1293EMG.hpp::update_adc_max()
    """
    if R2 == 4:
        fast_max = 0x8000
        precise_max = 0xF30000 if R3 in (6, 12) else 0x800000
    elif R2 == 5:
        fast_max = 0xC350
        precise_max = 0xB964F0 if R3 in (8, 16) else 0xC35000
    elif R2 == 6:
        fast_max = 0xF300
        precise_max = 0xE6A900 if R3 in (8, 16) else 0xF30000
    elif R2 == 8:
        fast_max = 0x8000
        precise_max = 0xF30000 if R3 in (6, 12) else 0x800000
    else:
        fast_max = 0x8000
        precise_max = 0x800000
    return fast_max, precise_max


def convert_precise(raw_24bit: int, adc_max: int) -> float:
    """Convert 24-bit raw ADC value to voltage.

    Formula from clvHd_module_ADS1293EMG.hpp::conv()
    """
    return (raw_24bit / adc_max - 0.5) * 4.8 / 3.5


def convert_fast(raw_16bit: int, adc_max: int = 0x8000) -> float:
    """Convert 16-bit raw ADC value to voltage."""
    return (raw_16bit / adc_max - 0.5) * 4.8 / 3.5


def compute_sample_rate(R1: int, R2: int, R3: int, high_freq: bool = False) -> float:
    """Compute effective sample rate from filter settings.

    base_freq = 204800 Hz (high_freq) or 102400 Hz (normal)
    sample_rate = base_freq / (R1 * R2 * R3)
    """
    base = 204800.0 if high_freq else 102400.0
    return base / (R1 * R2 * R3)


def compute_rld_cn(route: int = 0, bw_high: bool = False, cap_drive: int = 0) -> int:
    """Compute the RLD_CN (0x0C) register byte for the Right-Leg Drive.

    route:     0 = disabled, 1..6 = drive into IN1..IN6
    bw_high:   False = low BW (50 kHz), True = high BW (200 kHz)
    cap_drive: 0 = low, 1 = med-low, 2 = med-high, 3 = high
               (relevant when driving shielded cables)

    Bit layout (datasheet Figure 44):
      [7]    reserved
      [6]    RLD_BW         (0 low, 1 high)
      [5:4]  RLD_CAPDRIVE   (0..3)
      [3]    SHDN_RLD       (0 enabled, 1 shutdown)
      [2:0]  SELRLD         (0=off, 1..6=INx, 7 invalid)
    """
    if not 0 <= route <= 6:
        raise ValueError(f"rld route must be 0-6, got {route}")
    if not 0 <= cap_drive <= 3:
        raise ValueError(f"rld cap_drive must be 0-3, got {cap_drive}")
    val = 0
    if bw_high:
        val |= (1 << 6)
    val |= (cap_drive & 0x3) << 4
    if route == 0:
        val |= (1 << 3)        # SHDN_RLD = 1 (amp powered down)
    else:
        val |= (route & 0x07)  # SELRLD = route, SHDN_RLD = 0 (enabled)
    return val


# =============================================================================
# Default configuration
# =============================================================================

DEFAULT_CONFIG = {
    "channels": {
        "ch0": {"enabled": True, "pos_input": 1, "neg_input": 2},
        "ch1": {"enabled": True, "pos_input": 3, "neg_input": 4},
        "ch2": {"enabled": True, "pos_input": 5, "neg_input": 6},
    },
    "high_res": True,
    "high_freq": False,
    "filters": {
        "R1": 2,
        "R2": 4,
        "R3": 4,
    },
    "clock": "internal",
    "rld": {
        "route": 0,         # 0=off, 1..6=drive into IN1..IN6
        "bw_high": False,
        "cap_drive": 0,
    },
}
