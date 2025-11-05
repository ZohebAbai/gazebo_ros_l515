# Version Support Policy

## Current Version: 3.0.0 (Jazzy + Gazebo Harmonic)

This document clarifies which versions of ROS, Gazebo, and Ubuntu are supported by this package.

---

## ✅ Currently Supported

**This package ONLY works with:**

| Component | Version | Notes |
|-----------|---------|-------|
| **ROS** | ROS2 Jazzy Jalisco | Official support |
| **ROS** | ROS2 Rolling | Should work (untested) |
| **Gazebo** | Gazebo Harmonic (gz-sim 8.x) | Required |
| **Ubuntu** | Ubuntu 24.04 (Noble Numbat) | Required |

---

## ❌ No Longer Supported

**This package does NOT support:**

### ROS 1 (Discontinued)
- ❌ ROS Melodic
- ❌ ROS Noetic
- ❌ Any ROS 1 distribution

**Reason:** ROS 1 reached end-of-life. ROS2 is the future.

### Older ROS2 Distributions (Discontinued)
- ❌ ROS2 Foxy Fitzroy
- ❌ ROS2 Galactic Geochelone
- ❌ ROS2 Humble Hawksbill
- ❌ ROS2 Iron Irwini

**Reason:** These distributions use Gazebo Classic (gazebo11), not the new Gazebo (Gazebo Harmonic). The package has been simplified to support only Gazebo Harmonic.

### Gazebo Classic (Discontinued)
- ❌ Gazebo 11
- ❌ Gazebo 9
- ❌ Any version of Gazebo Classic

**Reason:** Gazebo Classic is being phased out in favor of the new Gazebo (formerly Ignition). ROS2 Jazzy officially uses Gazebo Harmonic.

### Older Ubuntu Versions (Discontinued)
- ❌ Ubuntu 22.04 (Jammy)
- ❌ Ubuntu 20.04 (Focal)
- ❌ Ubuntu 18.04 (Bionic)

**Reason:** ROS2 Jazzy requires Ubuntu 24.04.

---

## 🔄 Migration Path

### If you're using ROS 1:
1. **Stay on the old version** - Use the original ROS1 branch of this repository
2. **Upgrade to ROS2** - Migrate your robot to ROS2 Jazzy and use this package

### If you're using ROS2 Humble/Iron with Gazebo Classic:
1. **Use earlier release** - Check out an earlier commit before v3.0.0
2. **Upgrade to Jazzy** - Recommended for latest features and support

### Finding Old Versions:

**For ROS 1 (Melodic, Noetic):**
```bash
# Use the original main branch before ROS2 port
git clone https://github.com/ZohebAbai/gazebo_ros_l515.git
git checkout <commit-before-ros2-port>
```

**For ROS2 Humble/Iron with Gazebo Classic:**
```bash
# Use commits before Jazzy-only simplification
git clone https://github.com/ZohebAbai/gazebo_ros_l515.git
git checkout 80e708f  # Last commit with multi-distro support
```

---

## 📅 Version History

### Version 3.0.0 (Current)
- **Date:** 2025
- **Support:** ROS2 Jazzy + Gazebo Harmonic only
- **Changes:** Simplified to single distro/Gazebo version
- **Status:** ✅ Active development

### Version 2.0.0
- **Date:** 2025
- **Support:** ROS2 Foxy/Galactic/Humble/Iron + Gazebo Classic
- **Changes:** Multi-distro ROS2 support
- **Status:** ⚠️ No longer maintained

### Version 1.x
- **Date:** Pre-2025
- **Support:** ROS 1 (Melodic) + Gazebo Classic
- **Changes:** Original ROS1 implementation
- **Status:** ❌ Deprecated

---

## 🔮 Future Support Plans

### Planned Support:
- ✅ ROS2 Rolling (as it evolves)
- ✅ Future Gazebo versions (Garden+, when adopted by ROS2)
- ✅ Ubuntu 24.04+ (as new LTS versions release)

### No Plans for:
- ❌ Backporting to ROS 1
- ❌ Supporting Gazebo Classic
- ❌ Supporting older ROS2 distributions

---

## ❓ Why Drop Support for Older Versions?

### 1. **Maintenance Burden**
Supporting multiple ROS and Gazebo versions requires complex conditional logic, making the code harder to maintain and debug.

### 2. **ROS 1 End-of-Life**
ROS 1 Noetic reached end-of-life in May 2025. The robotics community is moving to ROS2.

### 3. **Gazebo Classic → New Gazebo**
The new Gazebo (Harmonic) is the future. Gazebo Classic is in maintenance mode only.

### 4. **API Differences**
Gazebo Classic and new Gazebo have significantly different APIs. Supporting both requires duplicate code paths.

### 5. **Focus on Quality**
By supporting only the latest versions, we can provide better quality, better documentation, and faster bug fixes.

---

## 🆘 Need Help with Older Versions?

### Option 1: Use Old Commits
Find the last commit that supported your version and use that.

### Option 2: Fork and Maintain
Fork this repository and maintain your own version for older ROS/Gazebo.

### Option 3: Community Support
Ask on ROS Discourse or GitHub Discussions - community members may help.

### Option 4: Professional Support
Contact the maintainer for professional support/consulting.

---

## 📊 Compatibility Matrix

| Your Setup | This Package | Recommendation |
|------------|--------------|----------------|
| ROS 1 + Gazebo Classic | ❌ Not compatible | Use original version or upgrade to ROS2 |
| ROS2 Humble + Gazebo Classic | ❌ Not compatible | Use commit `80e708f` or upgrade to Jazzy |
| ROS2 Iron + Gazebo Classic | ❌ Not compatible | Use commit `80e708f` or upgrade to Jazzy |
| ROS2 Jazzy + Gazebo Classic | ❌ Not compatible | Install Gazebo Harmonic |
| ROS2 Jazzy + Gazebo Harmonic | ✅ Compatible | Use current version (v3.0.0+) |
| ROS2 Rolling + Gazebo | ✅ Should work | Use current version (untested) |

---

## 📞 Contact

For questions about version support:
- **GitHub Issues:** https://github.com/ZohebAbai/gazebo_ros_l515/issues
- **Maintainer:** Zoheb Abai

---

## 🔐 License

Apache 2.0 - Same license for all versions
