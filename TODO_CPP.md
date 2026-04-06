Make sure that all of the code is fully working

Make sure that the vesc driver and all of the other sensor drivers are now fully benchmarked and tested so we know the exact ram/ cpu usages of each of the drivers


MAKE SURE TO REFACTOR GEOGRAPHIC_FUCNTION_LIBRARY LATER

ADD A microros agent binary and all of the stuff required to flash stuff to microros in the final .deb package



ADD A TABLE LIKE THIS TO THE DOCUMENTATION:

### 🚀 CI/CD Pipeline Summary
Our GitHub Actions pipeline is consolidated into a single workflow ([build-and-release.yml](.github/workflows/build-and-release.yml)) that handles testing, building, and deployment across all platforms.
| Feature | Pull Request (PR) | Push to `main` | Version Tag (`v*`) |
| :--- | :---: | :---: | :---: |
| **Verify Build** (amd64 / arm64) | ✅ | ✅ | ✅ |
| **Build & Download `.deb` Packages** | ✅ | ✅ | ✅ |
| **Push Images** (Docker Hub / GHCR) | ❌ | ✅ | ✅ |
| **Update Rolling Release** (`latest`) | ❌ | ✅ | ❌ |
| **Create Official Release** | ❌ | ❌ | ✅ |
| **Multi-Arch Manifests** | ❌ | ✅ | ✅ |
#### 🛠️ Key Pipeline Features:
- **Zero-Footprint Caching**: Uses a dynamic `.dockerignore` generated during the run to keep build contexts minimal (typically < 1MB).
- **Native GHA Caching**: Implements `type=gha,mode=max` caching for Docker, ensuring that independent layers (like dependencies and ROS installs) are only built when they change.
- **Hardware-Ready Artifacts**: Every run (including Pull Requests) generates downloadable Debian packages and pre-built tarballs for both `amd64` and `arm64` architectures.
- **Unified Logic**: One workflow handles the entire lifecycle from preliminary code checks to official multi-architecture releases.
