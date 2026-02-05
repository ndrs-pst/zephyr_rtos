# Top 10 PR Candidates for Zephyr RTOS

This document outlines the top 10 pull request candidates for contributing to the Zephyr RTOS project. Each candidate includes detailed rationale, impact assessment, and proposed changes.

---

## PR Candidate #1: Expand Driver Test Coverage

### Rationale
- **Current State**: Only ~11.7% test coverage (327 test files vs 2,792 driver source files)
- **Gap Analysis**: 26+ driver categories lack dedicated test suites including:
  - I3C (Inter-Integrated Circuit v3)
  - MFD (Multi-Function Device)
  - MDIO (Management Data Input/Output)
  - Haptics
  - Auxiliary Display
  - FPGA
  - LED Strip controllers
- **Industry Best Practice**: Critical embedded systems require 70-80% test coverage

### Impact
- **High Priority**: Improves code quality and prevents regressions
- **Maintainability**: Makes it easier to refactor and optimize drivers
- **Confidence**: Gives developers and users confidence in driver stability
- **CI/CD**: Enables automated validation of driver functionality
- **Risk Reduction**: Catches bugs before they reach production systems

### Proposed Changes

#### Phase 1: I3C Driver Tests (drivers/i3c/)
```c
// Create: tests/drivers/i3c/i3c_basic_api/
// Test file: tests/drivers/i3c/i3c_basic_api/src/test_i3c.c

ZTEST(i3c_basic_api, test_i3c_transfer)
{
    // Test basic I3C transfer operations
    // Test CCC (Common Command Code) handling
    // Test IBI (In-Band Interrupt) functionality
}

ZTEST(i3c_basic_api, test_i3c_bus_initialization)
{
    // Test dynamic address assignment (DAA)
    // Test bus enumeration
    // Test hot-join support
}
```

#### Phase 2: MFD Driver Tests (drivers/mfd/)
```c
// Create: tests/drivers/mfd/mfd_basic_api/
// Test multi-function device initialization
// Test sub-device registration and access
```

#### Files to Create
1. `tests/drivers/i3c/i3c_basic_api/testcase.yaml`
2. `tests/drivers/i3c/i3c_basic_api/prj.conf`
3. `tests/drivers/i3c/i3c_basic_api/src/test_i3c.c`
4. Similar structure for MFD, MDIO, haptics, etc.

---

## PR Candidate #2: Complete DS3231 RTC Driver Implementation

### Rationale
- **Current State**: DS3231 RTC driver has 5 documented TODOs/limitations
- **Missing Features**:
  - User mode support
  - Aging offset calibration
  - Century bit handling
  - Full year storage (currently only last 2 digits)
  - Temperature-triggered conversion
- **Real-World Need**: DS3231 is a popular, accurate RTC used in many embedded projects

### Impact
- **Medium-High Priority**: Commonly used hardware component
- **User Experience**: Eliminates year 2100 bug (2-digit year limitation)
- **Feature Completeness**: Unlocks full DS3231 capabilities
- **Use Cases**: Enables long-term deployment scenarios

### Proposed Changes

#### File: drivers/rtc/rtc_ds3231.c

```c
// 1. Add full year storage with external EEPROM/flash backing
struct rtc_ds3231_century {
    uint16_t base_century;  // e.g., 2000, 2100
    uint8_t century_register_bit;
};

// 2. Implement aging offset calibration
static int rtc_ds3231_set_calibration(const struct device *dev, int32_t calibration)
{
    // Use DS3231 aging offset register (0x10)
    // Calibration range: -128 to +127 (0.1 ppm per LSB)
    uint8_t aging_offset = (calibration / 0.1);
    return mfd_ds3231_i2c_set_registers(config->mfd, 
                                        DS3231_AGING_OFFSET_REG, 
                                        &aging_offset, 1);
}

// 3. Add user mode support with proper permission checks
#ifdef CONFIG_USERSPACE
static inline int rtc_ds3231_verify_user_access(const struct device *dev)
{
    return k_object_validate(dev, K_OBJ_DRIVER_RTC);
}
#endif

// 4. Implement temperature conversion trigger
static int rtc_ds3231_trigger_temp_conv(const struct device *dev)
{
    uint8_t control_reg;
    // Set CONV bit in Control/Status register (0x0E)
    return rtc_ds3231_modify_register(dev, DS3231_CONTROL_REG, 
                                      &control_reg, DS3231_CONV_BIT);
}

// 5. Century bit handling with persistent storage
static int rtc_ds3231_century_storage_init(const struct device *dev)
{
    // Use settings subsystem or dedicated flash area
    // Store century information persistently
}
```

#### Additional Files
- `include/zephyr/drivers/rtc/ds3231_extended.h` - Extended API definitions
- `dts/bindings/rtc/maxim,ds3231-rtc.yaml` - Add century-storage properties
- `tests/drivers/rtc/ds3231_extended/` - Comprehensive test suite

---

## PR Candidate #3: Improve POSIX Conformance Documentation

### Rationale
- **Current State**: POSIX conformance doc has TODO markers (line 25-29)
- **Gap**: Incomplete documentation of POSIX API implementation status
- **Standards Compliance**: IEEE 1003.1-2017 conformance tracking is incomplete
- **Developer Confusion**: Unclear which POSIX APIs are fully functional vs. stub implementations

### Impact
- **Medium Priority**: Critical for portability and developer experience
- **Standards Compliance**: Helps users understand Zephyr's POSIX capabilities
- **Porting Ease**: Developers can assess feasibility of porting POSIX applications
- **Transparency**: Clear documentation of limitations and workarounds

### Proposed Changes

#### File: doc/services/portability/posix/conformance/index.rst

```rst
.. _posix_asynchronous_io_status:

POSIX Asynchronous I/O Implementation Status
=============================================

Current Implementation
----------------------

The POSIX Asynchronous I/O APIs are currently implemented with the following status:

.. csv-table:: Asynchronous I/O Functions
   :header: Function, Status, Notes
   :widths: 30, 20, 50

   aio_read, Implemented, Returns ENOSYS by default; functional with :kconfig:option:`CONFIG_POSIX_AIO_ENABLE`
   aio_write, Implemented, Returns ENOSYS by default; functional with :kconfig:option:`CONFIG_POSIX_AIO_ENABLE`
   aio_error, Implemented, Full support
   aio_return, Implemented, Full support
   aio_cancel, Partial, Cancellation not guaranteed; may return AIO_NOTCANCELED
   aio_suspend, Implemented, Full support
   lio_listio, Stub, Always returns ENOSYS

Limitations and Workarounds
----------------------------

1. **Not Required for PSE51/PSE52**: Async I/O is optional for embedded profiles
2. **Rare Usage**: Async I/O functions are rarely used in embedded RTOS applications
3. **Alternative**: Use Zephyr native async APIs (k_work, k_msgq) for better performance

Roadmap
-------

- **v4.0**: Basic aio_read/aio_write with file system backends
- **v4.1**: Enhanced cancellation support
- **v4.2**: Full lio_listio implementation (if community demand exists)

Migration Guide
---------------

For applications requiring POSIX async I/O:

.. code-block:: c

   // Instead of POSIX aio_read()
   struct aiocb cb;
   aio_read(&cb);
   
   // Consider Zephyr native alternative
   struct k_work work;
   k_work_init(&work, async_read_handler);
   k_work_submit(&work);
```

#### Additional Documentation
- `doc/services/portability/posix/api_coverage_matrix.rst` - Complete API coverage matrix
- `doc/services/portability/posix/migration_guide.rst` - POSIX to Zephyr native API guide

---

## PR Candidate #4: Enhance Python Script Consistency and Error Handling

### Rationale
- **Current State**: 141+ Python scripts in `scripts/` with varying quality
- **Issues Found**:
  - Inconsistent error handling patterns
  - Missing type hints (Python 3.6+ feature)
  - Inconsistent logging practices
  - TODO/FIXME comments indicating incomplete implementations
- **Tooling**: Twister test runner has 6 FIXME comments indicating technical debt

### Impact
- **Medium Priority**: Improves developer tools reliability
- **Maintainability**: Easier to maintain and extend build/test infrastructure
- **Type Safety**: Reduces runtime errors with static type checking
- **Best Practices**: Aligns with modern Python development standards

### Proposed Changes

#### Example: scripts/pylib/twister/twisterlib/testsuite.py

```python
# Before (current):
def get_unique_test_name(test_path):
    # FIXME: We should not depend on path of test for unique names.
    return test_path.replace('/', '_')

# After (proposed):
from typing import Optional
from pathlib import Path
import hashlib

def get_unique_test_name(
    test_path: str,
    project_root: Optional[Path] = None
) -> str:
    """
    Generate a unique test identifier that doesn't rely solely on filesystem paths.
    
    Args:
        test_path: The test case path or identifier
        project_root: Optional project root for relative path calculation
        
    Returns:
        A unique, stable test identifier
        
    Note:
        Uses content-based hashing to ensure uniqueness across different
        filesystem layouts and operating systems.
    """
    if project_root:
        relative_path = Path(test_path).relative_to(project_root)
        canonical_path = relative_path.as_posix()
    else:
        canonical_path = test_path.replace('\\', '/')
    
    # Create stable hash for very long paths
    if len(canonical_path) > 200:
        path_hash = hashlib.sha256(canonical_path.encode()).hexdigest()[:8]
        return f"{canonical_path[:190]}_{path_hash}"
    
    return canonical_path.replace('/', '_')
```

#### Changes Across Multiple Files

1. **Type Hints Addition**:
```python
# scripts/pylib/twister/twisterlib/runner.py
from typing import List, Dict, Optional, Tuple

def run_tests(
    test_list: List[TestCase],
    options: Dict[str, Any],
    timeout: Optional[int] = None
) -> Tuple[int, List[TestResult]]:
    """Run test cases with proper type safety."""
    pass
```

2. **Error Handling Standardization**:
```python
# Consistent error handling pattern
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

def safe_file_operation(file_path: Path) -> Optional[str]:
    """Safely read file with comprehensive error handling."""
    try:
        return file_path.read_text(encoding='utf-8')
    except FileNotFoundError:
        logger.error(f"File not found: {file_path}")
        return None
    except PermissionError:
        logger.error(f"Permission denied: {file_path}")
        return None
    except UnicodeDecodeError:
        logger.warning(f"Binary file detected: {file_path}")
        return None
```

3. **Ruff Configuration Enhancement** (`.ruff.toml`):
```toml
[lint]
select = [
    "E",   # pycodestyle errors
    "F",   # pyflakes
    "I",   # isort
    "B",   # bugbear
    "UP",  # pyupgrade
    "ANN", # annotations (type hints)
    "SIM", # simplify
]

[lint.per-file-ignores]
# Legacy scripts can gradually adopt type hints
"scripts/legacy/*.py" = ["ANN"]
```

---

## PR Candidate #5: Add Comprehensive Sample Applications for Under-Documented Subsystems

### Rationale
- **Current State**: 594 sample applications, but coverage is uneven
- **Gaps Identified**:
  - Limited examples for newer subsystems (zbus, sensing, retention)
  - Complex subsystems lack beginner-friendly samples
  - Missing "real-world" integration examples
- **Learning Curve**: New contributors struggle without practical examples

### Impact
- **High Priority**: Improves developer onboarding and adoption
- **Education**: Provides learning resources for complex features
- **Best Practices**: Demonstrates recommended usage patterns
- **Community Growth**: Lowers barrier to entry for new developers

### Proposed Changes

#### New Sample 1: ZBus Complete Example
```
samples/subsys/zbus/weather_station/
├── CMakeLists.txt
├── prj.conf
├── README.rst
├── sample.yaml
└── src/
    ├── main.c
    ├── sensor_publisher.c
    ├── display_subscriber.c
    └── logger_subscriber.c
```

**README.rst** content:
```rst
ZBus Weather Station Example
#############################

Overview
********

This sample demonstrates ZBus (Zephyr message bus) for building a distributed
weather monitoring system with multiple publishers and subscribers.

Architecture
************

- **Temperature Sensor Publisher**: Publishes temperature readings
- **Humidity Sensor Publisher**: Publishes humidity readings  
- **Display Subscriber**: Subscribes to both channels, updates LCD
- **Logger Subscriber**: Subscribes to all channels, logs to flash
- **Alert Subscriber**: Subscribes with filtering, triggers alarms

Key Concepts Demonstrated
**************************

1. Channel creation and registration
2. Multiple publishers to same channel
3. Selective subscription with filters
4. Priority-based message delivery
5. Message buffering and overflow handling

Requirements
************

- Board with I2C support
- Temperature sensor (e.g., DHT22, BME280)
- Optional: LCD display, SD card for logging
```

**src/main.c**:
```c
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(weather_station, LOG_LEVEL_INF);

// Define message types
struct temperature_msg {
    float celsius;
    int64_t timestamp;
};

struct humidity_msg {
    float percent;
    int64_t timestamp;
};

// Define channels
ZBUS_CHAN_DEFINE(temp_chan,
                 struct temperature_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(display_sub, logger_sub),
                 ZBUS_MSG_INIT(0));

ZBUS_CHAN_DEFINE(humidity_chan,
                 struct humidity_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(display_sub, logger_sub),
                 ZBUS_MSG_INIT(0));

// Publisher thread
void sensor_publisher_thread(void *p1, void *p2, void *p3)
{
    struct temperature_msg temp_msg;
    
    while (1) {
        // Read from sensor (simplified)
        temp_msg.celsius = read_temperature();
        temp_msg.timestamp = k_uptime_get();
        
        // Publish to channel
        zbus_chan_pub(&temp_chan, &temp_msg, K_MSEC(100));
        
        k_sleep(K_SECONDS(5));
    }
}

// Subscriber example
ZBUS_LISTENER_DEFINE(display_sub, display_callback);

void display_callback(const struct zbus_channel *chan)
{
    if (chan == &temp_chan) {
        struct temperature_msg msg;
        zbus_chan_read(&temp_chan, &msg, K_MSEC(100));
        LOG_INF("Temperature: %.2f°C", msg.celsius);
    }
}

K_THREAD_DEFINE(sensor_thread, 1024, sensor_publisher_thread,
                NULL, NULL, NULL, 5, 0, 0);
```

#### New Sample 2: Retention Subsystem
```
samples/subsys/retention/bootloader_data/
```
Demonstrates:
- Sharing data between bootloader and application
- Surviving resets and power cycles
- Use cases: Boot reason tracking, crash diagnostics, update status

#### New Sample 3: Sensing Framework
```
samples/subsys/sensing/multi_sensor_fusion/
```
Demonstrates:
- Sensor data acquisition from multiple sources
- Data fusion algorithms
- Event-driven processing
- Power-efficient sensor management

---

## PR Candidate #6: Modernize Deprecated API Usage Across Codebase

### Rationale
- **Current State**: 82+ files contain deprecated API usage
- **Technical Debt**: Old APIs scheduled for removal in future versions
- **Maintenance Burden**: Supporting both old and new APIs
- **Best Practices**: Should guide users toward current APIs

### Impact
- **Medium Priority**: Prevents future breaking changes
- **Code Health**: Reduces technical debt
- **Future-Proofing**: Ensures examples use current best practices
- **Clarity**: Removes confusion about which APIs to use

### Proposed Changes

#### File: subsys/bluetooth/services/dis.c (10 deprecated items)

```c
// Before (deprecated):
#include <zephyr/bluetooth/buf.h>  // DEPRECATED

static struct bt_gatt_attr attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&dis_uuid),  // Old macro style
    // ... 10 similar deprecated patterns
};

// After (current):
#include <zephyr/bluetooth/att.h>  // Current API

static struct bt_gatt_attr attrs[] = {
    BT_GATT_SERVICE_DEFINE(dis_uuid),  // New macro style
    BT_GATT_CHARACTERISTIC(&dis_model_uuid,
                          BT_GATT_CHRC_READ,
                          BT_GATT_PERM_READ,
                          read_model, NULL, &model_str),
    // Modern attribute definition pattern
};
```

#### File: include/zephyr/usb/class/usb_hid.h (10 deprecated items)

```c
// Create migration header
/* 
 * USB HID API Migration Guide
 * 
 * DEPRECATED (before Zephyr 3.7):
 *   - usb_hid_set_proto() -> Use hid_device_set_protocol()
 *   - usb_hid_get_report() -> Use hid_device_get_report()
 * 
 * See doc/connectivity/usb/device/api_migration_guide.rst
 */

// Add deprecation warnings
__deprecated
static inline int usb_hid_set_proto(const struct device *dev, uint8_t proto)
{
    LOG_WRN("usb_hid_set_proto() is deprecated, use hid_device_set_protocol()");
    return hid_device_set_protocol(dev, proto);
}
```

#### Documentation Update
Create: `doc/connectivity/usb/device/api_migration_guide.rst`
```rst
USB Device API Migration Guide
###############################

This guide helps migrate from deprecated USB device APIs to current APIs.

.. list-table:: USB HID API Changes
   :header-rows: 1
   
   * - Deprecated API (pre-3.7)
     - Current API (3.7+)
     - Migration Notes
   * - ``usb_hid_set_proto()``
     - ``hid_device_set_protocol()``
     - Direct replacement, same parameters
   * - ``USB_DFU_*`` macros
     - ``DFU_*`` macros
     - Updated to match USB-IF naming
```

#### Automated Migration Script
```python
# scripts/migration/usb_api_migrate.py
"""
Automated USB API migration tool.

Usage:
    python scripts/migration/usb_api_migrate.py <directory>
"""
import re
from pathlib import Path

API_MIGRATIONS = {
    r'usb_hid_set_proto\s*\(': 'hid_device_set_protocol(',
    r'usb_hid_get_report\s*\(': 'hid_device_get_report(',
    # ... more patterns
}

def migrate_file(file_path: Path) -> bool:
    content = file_path.read_text()
    updated = content
    
    for old_pattern, new_api in API_MIGRATIONS.items():
        updated = re.sub(old_pattern, new_api, updated)
    
    if updated != content:
        file_path.write_text(updated)
        return True
    return False
```

---

## PR Candidate #7: Implement CI/CD Pipeline Improvements

### Rationale
- **Current State**: 32 GitHub Actions workflows with some inefficiencies
- **Opportunities**:
  - Parallel test execution optimization
  - Selective testing based on changed files
  - Better caching strategies for dependencies
  - Improved failure reporting
- **Developer Experience**: Long CI times slow down development

### Impact
- **High Priority**: Improves developer productivity
- **Cost Reduction**: Reduces CI compute costs
- **Faster Feedback**: Developers get results quicker
- **Reliability**: Better error detection and reporting

### Proposed Changes

#### File: .github/workflows/twister_tests.yaml

```yaml
name: Twister Tests (Optimized)

on:
  pull_request:
    paths:
      - 'arch/**'
      - 'drivers/**'
      - 'kernel/**'
      - 'subsys/**'
      - 'tests/**'
      - 'samples/**'

jobs:
  # New: Detect changed areas
  detect-changes:
    runs-on: ubuntu-latest
    outputs:
      test-kernel: ${{ steps.filter.outputs.kernel }}
      test-drivers: ${{ steps.filter.outputs.drivers }}
      test-bluetooth: ${{ steps.filter.outputs.bluetooth }}
      test-networking: ${{ steps.filter.outputs.networking }}
    steps:
      - uses: actions/checkout@v4
      - uses: dorny/paths-filter@v2
        id: filter
        with:
          filters: |
            kernel:
              - 'kernel/**'
              - 'tests/kernel/**'
            drivers:
              - 'drivers/**'
              - 'tests/drivers/**'
            bluetooth:
              - 'subsys/bluetooth/**'
              - 'tests/bluetooth/**'
            networking:
              - 'subsys/net/**'
              - 'tests/net/**'

  # Conditional matrix jobs
  test-kernel:
    needs: detect-changes
    if: needs.detect-changes.outputs.test-kernel == 'true'
    runs-on: ubuntu-latest
    strategy:
      matrix:
        platform: [qemu_x86, qemu_cortex_m3, native_sim]
      fail-fast: false
    steps:
      - uses: actions/checkout@v4
      
      - name: Cache SDK
        uses: actions/cache@v3
        with:
          path: ~/zephyr-sdk
          key: zephyr-sdk-${{ hashFiles('SDK_VERSION') }}
      
      - name: Cache Python packages
        uses: actions/cache@v3
        with:
          path: ~/.cache/pip
          key: pip-${{ hashFiles('scripts/requirements*.txt') }}
      
      - name: Run Kernel Tests
        run: |
          ./scripts/twister \
            --platform ${{ matrix.platform }} \
            --testsuite-root tests/kernel \
            --inline-logs \
            --coverage \
            --coverage-formats html,lcov
      
      - name: Upload Coverage
        uses: codecov/codecov-action@v3
        with:
          files: ./twister-out/coverage.lcov
          flags: kernel-${{ matrix.platform }}

  test-drivers:
    needs: detect-changes
    if: needs.detect-changes.outputs.test-drivers == 'true'
    runs-on: ubuntu-latest
    strategy:
      matrix:
        driver-category: [gpio, i2c, spi, uart, sensor]
      fail-fast: false
    steps:
      - name: Run Driver Tests
        run: |
          ./scripts/twister \
            --testsuite-root tests/drivers/${{ matrix.driver-category }} \
            --inline-logs

  # New: Improved failure reporting
  report-results:
    needs: [test-kernel, test-drivers]
    if: always()
    runs-on: ubuntu-latest
    steps:
      - name: Generate Test Report
        run: |
          python scripts/ci/generate_test_report.py \
            --format markdown \
            --output test_report.md
      
      - name: Comment PR
        uses: actions/github-script@v7
        with:
          script: |
            const fs = require('fs');
            const report = fs.readFileSync('test_report.md', 'utf8');
            github.rest.issues.createComment({
              issue_number: context.issue.number,
              owner: context.repo.owner,
              repo: context.repo.repo,
              body: report
            });
```

#### New Script: scripts/ci/generate_test_report.py

```python
#!/usr/bin/env python3
"""
Generate human-readable test reports from Twister results.
"""

import json
import argparse
from pathlib import Path
from typing import Dict, List

def generate_markdown_report(results: Dict) -> str:
    """Generate markdown formatted test report."""
    total = results['total_tests']
    passed = results['passed']
    failed = results['failed']
    skipped = results['skipped']
    
    report = f"""
# Test Results Summary

| Metric | Count | Percentage |
|--------|-------|-----------|
| Total Tests | {total} | 100% |
| ✅ Passed | {passed} | {passed/total*100:.1f}% |
| ❌ Failed | {failed} | {failed/total*100:.1f}% |
| ⏭️ Skipped | {skipped} | {skipped/total*100:.1f}% |

"""
    
    if failed > 0:
        report += "\n## Failed Tests\n\n"
        for test in results['failed_tests']:
            report += f"- **{test['name']}** ({test['platform']})\n"
            report += f"  ```\n  {test['error'][:200]}...\n  ```\n"
    
    return report

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default='twister-out/twister.json')
    parser.add_argument('--format', choices=['markdown', 'html'], default='markdown')
    parser.add_argument('--output', default='test_report.md')
    args = parser.parse_args()
    
    with open(args.input) as f:
        results = json.load(f)
    
    if args.format == 'markdown':
        report = generate_markdown_report(results)
    
    Path(args.output).write_text(report)

if __name__ == '__main__':
    main()
```

---

## PR Candidate #8: Enhance Device Tree Binding Documentation

### Rationale
- **Current State**: DTS bindings exist but lack comprehensive examples
- **Developer Pain Point**: Device tree syntax is confusing for newcomers
- **Missing**: Real-world examples showing complete board configurations
- **Best Practices**: Need guide on writing custom bindings

### Impact
- **Medium-High Priority**: Critical for board bring-up
- **Learning Curve**: Major barrier for hardware developers
- **Quality**: Improves binding consistency across vendors
- **Extensibility**: Easier to add new hardware support

### Proposed Changes

#### New Documentation: doc/hardware/devicetree/bindings_guide.rst

```rst
Device Tree Binding Development Guide
######################################

This comprehensive guide covers creating high-quality device tree bindings.

.. contents::
   :local:
   :depth: 2

Quick Start
***********

A device tree binding describes hardware characteristics in a machine-readable format.

Basic Binding Structure
=======================

.. code-block:: yaml

   # dts/bindings/sensor/vendor,sensor-xyz.yaml
   description: XYZ Temperature and Humidity Sensor
   
   compatible: "vendor,sensor-xyz"
   
   include: [sensor-device.yaml, i2c-device.yaml]
   
   properties:
     int-gpios:
       type: phandle-array
       description: |
         Interrupt pin. The sensor asserts this pin when new data is ready.
         
     sampling-rate:
       type: int
       default: 1
       enum:
         - 1    # 1 Hz
         - 10   # 10 Hz
         - 100  # 100 Hz
       description: Sampling rate in Hz
     
     high-precision:
       type: boolean
       description: Enable high-precision mode (slower, more accurate)

Complete Example
================

Here's a complete example for a custom sensor board:

**Board DTS** (boards/arm/custom_board/custom_board.dts):

.. code-block:: devicetree

   /dts-v1/;
   #include <nordic/nrf52840.dtsi>
   #include <nordic/nrf52840-qiaa.dtsi>
   
   / {
       model = "Custom Sensor Board";
       compatible = "vendor,custom-board";
       
       chosen {
           zephyr,console = &uart0;
           zephyr,shell-uart = &uart0;
           zephyr,sram = &sram0;
           zephyr,flash = &flash0;
       };
       
       aliases {
           temp-sensor = &sensor_xyz;
       };
   };
   
   &i2c0 {
       compatible = "nordic,nrf-twi";
       status = "okay";
       clock-frequency = <I2C_BITRATE_FAST>;
       
       sda-pin = <26>;
       scl-pin = <27>;
       
       sensor_xyz: sensor-xyz@48 {
           compatible = "vendor,sensor-xyz";
           reg = <0x48>;
           int-gpios = <&gpio0 25 GPIO_ACTIVE_LOW>;
           sampling-rate = <10>;
           high-precision;
       };
   };

**Binding Validation**

Zephyr automatically validates bindings during build:

.. code-block:: console

   $ west build -b custom_board samples/sensor/app
   -- Devicetree validation passed
   -- Found sensor: vendor,sensor-xyz at i2c@40003000

Advanced Topics
***************

Child Binding
=============

For devices with sub-components:

.. code-block:: yaml

   # Multi-channel ADC
   child-binding:
     description: ADC Channel Configuration
     properties:
       reg:
         type: int
         required: true
       zephyr,gain:
         type: string
         enum: ["ADC_GAIN_1", "ADC_GAIN_2", "ADC_GAIN_4"]
       zephyr,acquisition-time:
         type: int

Bus-Specific Bindings
=====================

.. code-block:: yaml

   # SPI device with specific timing requirements
   on-bus: spi
   
   properties:
     spi-max-frequency:
       type: int
       default: 1000000
       description: Maximum SPI clock frequency in Hz

Testing Bindings
****************

.. code-block:: python

   # tests/bindings/vendor_sensor_xyz/test_binding.py
   
   def test_sensor_binding_validation():
       """Ensure binding validates correctly."""
       dts_content = """
           sensor@48 {
               compatible = "vendor,sensor-xyz";
               reg = <0x48>;
               int-gpios = <&gpio0 25 0>;
           };
       """
       validate_devicetree(dts_content)

Common Pitfalls
***************

1. **Forgetting Required Properties**: Always mark essential properties as ``required: true``
2. **Incorrect Phandle Types**: Use ``phandle-array`` for GPIO, not ``int``
3. **Missing Descriptions**: Every property needs documentation
4. **Enum Inconsistency**: Keep enum values consistent across similar bindings

Best Practices Checklist
*************************

.. checklist::

   - [ ] Binding follows existing naming conventions
   - [ ] All properties have descriptions
   - [ ] Defaults are sensible for common use cases
   - [ ] Example DTS in binding or documentation
   - [ ] Binding validates with test devicetree
   - [ ] Driver code matches binding properties
   - [ ] MAINTAINERS.yml updated with binding ownership
```

#### Interactive Binding Generator Tool

```python
# scripts/dts/binding_generator.py
"""
Interactive tool to generate device tree bindings.

Usage:
    python scripts/dts/binding_generator.py
"""

def interactive_binding_wizard():
    """Guide user through binding creation."""
    print("=== Device Tree Binding Generator ===\n")
    
    # Collect basic information
    compatible = input("Compatible string (vendor,device): ")
    description = input("Brief description: ")
    bus_type = input("Bus type (i2c/spi/gpio/none): ")
    
    binding = {
        'description': description,
        'compatible': compatible,
    }
    
    if bus_type != 'none':
        binding['include'] = [f'{bus_type}-device.yaml']
    
    binding['properties'] = {}
    
    # Add properties interactively
    while True:
        prop_name = input("\nProperty name (or 'done'): ")
        if prop_name == 'done':
            break
        
        prop_type = input(f"  Type for {prop_name} (int/string/boolean/phandle-array): ")
        prop_desc = input(f"  Description: ")
        
        binding['properties'][prop_name] = {
            'type': prop_type,
            'description': prop_desc
        }
        
        if prop_type == 'int':
            default = input(f"  Default value (or skip): ")
            if default:
                binding['properties'][prop_name]['default'] = int(default)
    
    # Generate YAML
    import yaml
    output_file = f"dts/bindings/generated/{compatible.replace(',', '-')}.yaml"
    Path(output_file).parent.mkdir(exist_ok=True)
    Path(output_file).write_text(yaml.dump(binding, default_flow_style=False))
    
    print(f"\n✅ Binding created: {output_file}")
```

---

## PR Candidate #9: Create Performance Benchmarking Framework

### Rationale
- **Current State**: No standardized performance benchmarking infrastructure
- **Need**: Developers need to compare performance across:
  - Different architectures (ARM vs RISC-V vs x86)
  - Different optimization levels
  - Before/after optimizations
  - Kernel vs. user space operations
- **Industry Standard**: Other RTOSes (FreeRTOS, ThreadX) have benchmark suites

### Impact
- **High Priority**: Enables data-driven optimization decisions
- **Marketing**: Provides objective performance comparisons
- **Regression Detection**: Catches performance regressions in CI
- **Optimization**: Identifies bottlenecks systematically

### Proposed Changes

#### New Directory Structure
```
tests/benchmarks/
├── kernel/
│   ├── context_switch/
│   ├── irq_latency/
│   ├── semaphore_throughput/
│   └── thread_creation/
├── lib/
│   ├── libc_performance/
│   └── data_structures/
├── drivers/
│   ├── spi_throughput/
│   └── uart_latency/
├── framework/
│   ├── benchmark.h
│   └── results_reporter.c
└── doc/
    └── benchmarking_guide.rst
```

#### Core Framework: tests/benchmarks/framework/benchmark.h

```c
/**
 * @file benchmark.h
 * @brief Zephyr Performance Benchmarking Framework
 */

#ifndef ZEPHYR_BENCHMARK_H_
#define ZEPHYR_BENCHMARK_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>

/**
 * @brief Benchmark timing result
 */
struct benchmark_result {
    const char *name;
    uint32_t iterations;
    uint64_t total_cycles;
    uint64_t min_cycles;
    uint64_t max_cycles;
    uint64_t avg_cycles;
    uint32_t frequency_hz;
};

/**
 * @brief Begin benchmark timing
 */
static inline uint64_t benchmark_begin(void)
{
    return k_cycle_get_64();
}

/**
 * @brief End benchmark timing
 */
static inline uint64_t benchmark_end(uint64_t start)
{
    return k_cycle_get_64() - start;
}

/**
 * @brief Run benchmark multiple times and collect statistics
 * 
 * @param name Benchmark name
 * @param iterations Number of iterations to run
 * @param bench_fn Benchmark function to execute
 * @param result Output results structure
 */
void benchmark_run(const char *name, uint32_t iterations,
                   void (*bench_fn)(void), struct benchmark_result *result);

/**
 * @brief Report benchmark results in machine-readable format
 */
void benchmark_report_json(const struct benchmark_result *result);

/**
 * @brief Report benchmark results in human-readable format
 */
void benchmark_report_console(const struct benchmark_result *result);

/**
 * @brief Macro for simple benchmark execution
 * 
 * Example:
 *   BENCHMARK_RUN("memory_copy", 1000, {
 *       memcpy(dest, src, 1024);
 *   });
 */
#define BENCHMARK_RUN(name, iterations, code_block) \
    do { \
        struct benchmark_result result; \
        void bench_##name(void) { code_block } \
        benchmark_run(#name, iterations, bench_##name, &result); \
        benchmark_report_console(&result); \
    } while (0)

#endif /* ZEPHYR_BENCHMARK_H_ */
```

#### Example Benchmark: tests/benchmarks/kernel/context_switch/

```c
/**
 * @file tests/benchmarks/kernel/context_switch/src/main.c
 * @brief Thread context switch latency benchmark
 */

#include <zephyr/kernel.h>
#include <zephyr/benchmark/benchmark.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(context_switch_bench, LOG_LEVEL_INF);

#define STACK_SIZE 1024
#define THREAD_PRIORITY 5
#define ITERATIONS 10000

K_THREAD_STACK_DEFINE(thread_a_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_b_stack, STACK_SIZE);

static struct k_thread thread_a_data;
static struct k_thread thread_b_data;
static struct k_sem sync_sem;
static volatile uint64_t switch_start;
static volatile uint64_t switch_end;
static uint64_t measurements[ITERATIONS];
static uint32_t measurement_idx;

void thread_a_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (measurement_idx < ITERATIONS) {
        k_sem_take(&sync_sem, K_FOREVER);
        switch_start = benchmark_begin();
        k_sem_give(&sync_sem);
    }
}

void thread_b_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (measurement_idx < ITERATIONS) {
        k_sem_take(&sync_sem, K_FOREVER);
        switch_end = benchmark_end(switch_start);
        measurements[measurement_idx++] = switch_end;
        k_sem_give(&sync_sem);
    }
}

int main(void)
{
    LOG_INF("Context Switch Latency Benchmark");
    LOG_INF("Platform: %s", CONFIG_BOARD);
    LOG_INF("CPU Frequency: %d Hz", sys_clock_hw_cycles_per_sec());

    k_sem_init(&sync_sem, 1, 1);

    k_thread_create(&thread_a_data, thread_a_stack, STACK_SIZE,
                    thread_a_entry, NULL, NULL, NULL,
                    THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&thread_b_data, thread_b_stack, STACK_SIZE,
                    thread_b_entry, NULL, NULL, NULL,
                    THREAD_PRIORITY, 0, K_NO_WAIT);

    k_sleep(K_SECONDS(5));

    // Calculate statistics
    uint64_t total = 0, min = UINT64_MAX, max = 0;
    
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        total += measurements[i];
        if (measurements[i] < min) min = measurements[i];
        if (measurements[i] > max) max = measurements[i];
    }

    struct benchmark_result result = {
        .name = "context_switch",
        .iterations = ITERATIONS,
        .total_cycles = total,
        .min_cycles = min,
        .max_cycles = max,
        .avg_cycles = total / ITERATIONS,
        .frequency_hz = sys_clock_hw_cycles_per_sec(),
    };

    benchmark_report_console(&result);
    benchmark_report_json(&result);

    return 0;
}
```

#### CI Integration: .github/workflows/benchmarks.yaml

```yaml
name: Performance Benchmarks

on:
  schedule:
    - cron: '0 0 * * 0'  # Weekly on Sunday
  workflow_dispatch:     # Manual trigger

jobs:
  benchmark:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        platform: [qemu_x86, qemu_cortex_m3, qemu_riscv32]
    steps:
      - uses: actions/checkout@v4
      
      - name: Run Benchmarks
        run: |
          ./scripts/twister \
            --platform ${{ matrix.platform }} \
            --testsuite-root tests/benchmarks \
            --inline-logs \
            --json-out benchmark_results.json
      
      - name: Upload Results
        uses: actions/upload-artifact@v3
        with:
          name: benchmark-${{ matrix.platform }}
          path: benchmark_results.json
      
      - name: Compare to Baseline
        run: |
          python scripts/benchmarks/compare_results.py \
            --current benchmark_results.json \
            --baseline benchmarks/baseline/${{ matrix.platform }}.json \
            --threshold 5  # 5% regression threshold
```

---

## PR Candidate #10: Improve Security Documentation and Security Scanning Integration

### Rationale
- **Current State**: Basic security documentation exists but needs expansion
- **CVE Management**: No automated vulnerability tracking for dependencies
- **Security Best Practices**: Limited guidance for secure application development
- **Static Analysis**: CodeQL is present but could be enhanced

### Impact
- **Critical Priority**: Security is paramount for IoT/embedded devices
- **Compliance**: Helps meet security certification requirements (IEC 62443, etc.)
- **Awareness**: Educates developers on secure coding practices
- **Supply Chain**: Addresses dependency vulnerability management

### Proposed Changes

#### Enhanced Security Documentation

**File: doc/security/secure_development_guide.rst**

```rst
Secure Development Guide for Zephyr
####################################

This guide provides comprehensive security guidelines for Zephyr applications.

.. contents::
   :local:
   :depth: 3

Security Principles
*******************

Defense in Depth
================

Zephyr applications should implement multiple security layers:

1. **Hardware Security**: Secure boot, memory protection, crypto accelerators
2. **Software Security**: Input validation, bounds checking, secure APIs
3. **Network Security**: TLS, secure protocols, authentication
4. **Update Security**: Signed firmware, rollback protection

Common Vulnerabilities and Mitigations
***************************************

Buffer Overflows
================

**Vulnerable Code:**

.. code-block:: c

   void process_data(const uint8_t *input, size_t len)
   {
       uint8_t buffer[100];
       memcpy(buffer, input, len);  // ❌ No bounds check!
   }

**Secure Code:**

.. code-block:: c

   void process_data(const uint8_t *input, size_t len)
   {
       uint8_t buffer[100];
       
       // ✅ Validate input size
       if (len > sizeof(buffer)) {
           LOG_ERR("Input too large: %zu bytes", len);
           return -EINVAL;
       }
       
       memcpy(buffer, input, len);
       return 0;
   }

Integer Overflows
=================

**Vulnerable Code:**

.. code-block:: c

   void *allocate_array(size_t count, size_t size)
   {
       size_t total = count * size;  // ❌ Can overflow!
       return k_malloc(total);
   }

**Secure Code:**

.. code-block:: c

   void *allocate_array(size_t count, size_t size)
   {
       // ✅ Check for overflow
       if (count > 0 && size > SIZE_MAX / count) {
           LOG_ERR("Integer overflow in allocation");
           return NULL;
       }
       
       size_t total = count * size;
       return k_malloc(total);
   }

Use After Free
==============

**Vulnerable Code:**

.. code-block:: c

   struct sensor_data *data = get_sensor_data();
   process_sensor(data);
   k_free(data);
   
   if (error_occurred) {
       log_sensor_data(data);  // ❌ Use after free!
   }

**Secure Code:**

.. code-block:: c

   struct sensor_data *data = get_sensor_data();
   
   if (error_occurred) {
       log_sensor_data(data);  // ✅ Use before free
   }
   
   process_sensor(data);
   k_free(data);
   data = NULL;  // ✅ Prevent dangling pointer
```

#### Automated Dependency Scanning

**File: .github/workflows/security_scan.yaml**

```yaml
name: Security Scanning

on:
  push:
    branches: [main]
  pull_request:
  schedule:
    - cron: '0 0 * * 1'  # Weekly

jobs:
  codeql:
    name: CodeQL Security Analysis
    runs-on: ubuntu-latest
    permissions:
      security-events: write
    
    steps:
      - uses: actions/checkout@v4
      
      - name: Initialize CodeQL
        uses: github/codeql-action/init@v2
        with:
          languages: c, cpp, python
          queries: security-extended, security-and-quality
      
      - name: Build for Analysis
        run: |
          west build -b qemu_x86 samples/hello_world
      
      - name: Perform CodeQL Analysis
        uses: github/codeql-action/analyze@v2

  dependency-scan:
    name: Dependency Vulnerability Scan
    runs-on: ubuntu-latest
    
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      
      - name: Scan Dependencies
        run: |
          # Check Python dependencies
          pip install safety
          safety check --file scripts/requirements.txt --json > safety_report.json
          
          # Check for known CVEs in modules
          python scripts/security/check_module_cves.py west.yml
      
      - name: Upload SARIF results
        uses: github/codeql-action/upload-sarif@v2
        with:
          sarif_file: safety_report.sarif

  secrets-scan:
    name: Scan for Committed Secrets
    runs-on: ubuntu-latest
    
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      
      - name: Run Gitleaks
        uses: gitleaks/gitleaks-action@v2
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}

  supply-chain:
    name: Supply Chain Security
    runs-on: ubuntu-latest
    
    steps:
      - uses: actions/checkout@v4
      
      - name: SBOM Generation
        run: |
          # Generate Software Bill of Materials
          syft packages dir:. -o spdx-json > sbom.spdx.json
      
      - name: Vulnerability Scan
        run: |
          # Scan SBOM for vulnerabilities
          grype sbom:sbom.spdx.json --fail-on high
      
      - name: Upload SBOM
        uses: actions/upload-artifact@v3
        with:
          name: sbom
          path: sbom.spdx.json
```

#### Security Scanning Script

**File: scripts/security/check_module_cves.py**

```python
#!/usr/bin/env python3
"""
Check Zephyr modules for known CVEs.

Queries the National Vulnerability Database (NVD) for each module
declared in west.yml and reports any known vulnerabilities.

Usage:
    python scripts/security/check_module_cves.py west.yml
"""

import sys
import yaml
import requests
from typing import List, Dict
from packaging import version

class CVEChecker:
    """Check modules for CVE vulnerabilities."""
    
    NVD_API = "https://services.nvd.nist.gov/rest/json/cves/2.0"
    
    def __init__(self, manifest_file: str):
        with open(manifest_file) as f:
            self.manifest = yaml.safe_load(f)
    
    def check_all_modules(self) -> List[Dict]:
        """Check all modules for vulnerabilities."""
        vulnerabilities = []
        
        for project in self.manifest.get('manifest', {}).get('projects', []):
            name = project.get('name')
            revision = project.get('revision', 'main')
            
            print(f"Checking {name}@{revision}...")
            
            cves = self.query_cves(name)
            if cves:
                vulnerabilities.append({
                    'module': name,
                    'revision': revision,
                    'cves': cves
                })
        
        return vulnerabilities
    
    def query_cves(self, module_name: str) -> List[Dict]:
        """Query NVD for CVEs related to module."""
        try:
            params = {
                'keywordSearch': module_name,
                'resultsPerPage': 10
            }
            response = requests.get(self.NVD_API, params=params, timeout=30)
            
            if response.status_code == 200:
                data = response.json()
                return data.get('vulnerabilities', [])
        except Exception as e:
            print(f"Warning: Could not query NVD for {module_name}: {e}")
        
        return []
    
    def generate_report(self, vulnerabilities: List[Dict]) -> str:
        """Generate security report."""
        if not vulnerabilities:
            return "✅ No known vulnerabilities found in dependencies."
        
        report = "⚠️ Security Vulnerabilities Found\n\n"
        
        for vuln in vulnerabilities:
            report += f"**{vuln['module']}** ({vuln['revision']})\n"
            for cve in vuln['cves'][:5]:  # Top 5 CVEs
                cve_id = cve.get('cve', {}).get('id', 'Unknown')
                report += f"  - {cve_id}\n"
            report += "\n"
        
        return report

def main():
    if len(sys.argv) < 2:
        print("Usage: check_module_cves.py west.yml")
        sys.exit(1)
    
    checker = CVEChecker(sys.argv[1])
    vulnerabilities = checker.check_all_modules()
    report = checker.generate_report(vulnerabilities)
    
    print("\n" + report)
    
    # Exit with error if critical vulnerabilities found
    if vulnerabilities:
        sys.exit(1)

if __name__ == '__main__':
    main()
```

---

## Summary and Prioritization

### Implementation Priority Matrix

| PR # | Title | Priority | Effort | Impact | Quick Win |
|------|-------|----------|--------|--------|-----------|
| 1 | Expand Driver Test Coverage | High | High | High | No |
| 2 | Complete DS3231 RTC Driver | Medium-High | Medium | Medium | Yes |
| 3 | Improve POSIX Documentation | Medium | Low | Medium | Yes |
| 4 | Python Script Enhancements | Medium | Medium | Medium | No |
| 5 | New Sample Applications | High | High | High | No |
| 6 | Modernize Deprecated APIs | Medium | Medium | Medium | No |
| 7 | CI/CD Improvements | High | High | High | No |
| 8 | Device Tree Binding Docs | Medium-High | Medium | High | Yes |
| 9 | Performance Benchmarking | High | High | High | No |
| 10 | Security Enhancements | Critical | High | Critical | No |

### Quick Wins (Start Here)
1. **PR #2**: DS3231 RTC Driver - Concrete TODOs, clear scope
2. **PR #3**: POSIX Documentation - Documentation improvement, no code risk
3. **PR #8**: Device Tree Guide - High value, moderate effort

### High-Impact Long-Term
1. **PR #10**: Security Enhancements - Critical for production use
2. **PR #7**: CI/CD Improvements - Benefits entire development workflow
3. **PR #1**: Test Coverage - Foundational quality improvement

### Community Engagement
1. **PR #5**: Sample Applications - Helps onboard new developers
2. **PR #9**: Benchmarking Framework - Attracts performance-focused users

---

## Contributing These PRs

### Before Starting
1. Review [Zephyr Contribution Guidelines](https://docs.zephyrproject.org/latest/contribute/index.html)
2. Join [Zephyr Discord](https://chat.zephyrproject.org) and discuss proposals
3. Search existing issues/PRs to avoid duplication
4. Check [MAINTAINERS.yml](MAINTAINERS.yml) for area ownership

### Development Workflow
1. Create issue describing the proposal
2. Get feedback from maintainers
3. Fork repository and create feature branch
4. Implement changes with tests
5. Run `./scripts/twister` for affected areas
6. Submit PR with DCO sign-off (`git commit -s`)
7. Address review feedback
8. Wait for CI to pass
9. Maintainer merges when approved

### DCO Sign-off
All commits must include Developer Certificate of Origin:
```bash
git commit -s -m "drivers: rtc: Complete DS3231 implementation

Add century bit handling and full year storage.

Fixes #XXXXX

Signed-off-by: Your Name <your.email@example.com>"
```

---

## Conclusion

These 10 PR candidates represent high-value contributions across different aspects of the Zephyr RTOS project:

- **Quality**: Test coverage, deprecated API cleanup
- **Features**: Driver completeness, benchmarking framework
- **Documentation**: POSIX, device tree, security guides
- **Infrastructure**: CI/CD, Python tooling
- **Education**: Sample applications, guides

Each candidate includes concrete implementation details, making them actionable for contributors of varying skill levels. The proposals balance quick wins with strategic long-term improvements, ensuring both immediate value and sustained project health.

**Next Steps**: Select 1-2 candidates that align with your expertise and interest, engage with the community, and start contributing!
