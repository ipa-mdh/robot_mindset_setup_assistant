# Robot Mindeset Setup Assistant

`robot_mindset_setup_assistant` is a modular CLI tool designed to streamline the development of robotics software, particularly in ROS environments. It automates the creation of standardized ROS packages, interfaces, Docker setups, documentation, and CI/CD configurations — ensuring consistency, reducing boilerplate, and enabling scalable collaboration in team settings.

---

## 🎯 Purpose & Vision

Modern robotics projects often suffer from fragmented styles, inconsistent folder structures, and a lack of unified tools for managing packages, interfaces, and deployment. This tool aims to:

- Automate repetitive coding patterns (setup, interfaces, docs, Docker, CI)
- Standardize package structures across teams and projects
- Introduce a default interface model for ROS: topics, services, actions
- Integrate development lifecycle elements: launch files, documentation, git workflows
- Lay the groundwork for a future UI (NiceGUI) and project management dashboard

Whether you are a ROS newcomer or a robotics research team deploying multi-package systems, this tool helps you stay organized and move faster.

---

## 🛠️ Features

### 🧱 Code & Package Generation
- Create ROS packages in **C++** or **Python**
- Inject modular interfaces: **topics**, **services**, **actions**
- Auto-generate:
  - `CMakeLists.txt`, `package.xml`
  - ROS launch files (compatible with ROS 1 XML and ROS 2 Python-style)
  - Instance configurations per node
  - Package documentation with **Doxygen** and **doxygen-awesome-css**

### 🐳 Docker-Based Dev Setup
- Generates:
  - `Dockerfile.base`: minimal image with dependencies installed
  - `Dockerfile.devcontainer`: development environment for VS Code Remote Containers
    - Optional NoVNC + Virtual Desktop setup for remote GUIs
  - `Dockerfile.ci`: GitLab CI-ready with git credentials handling
  - `Dockerfile.run`: for deploying stripped, runnable containers
- Supports caching for fast builds

### 🔧 GitLab CI/CD Integration
- GitLab `.gitlab-ci.yml` stages generated via `dev-setup`
  - `dependencies`, `build`, `test`, `doc`
- CI tokens and credentials injected securely

### 📦 Resource Management
- Handles dependencies via:
  - `rosdep`
  - `apt`
  - `pip`
  - `vcstool` (`.rosinstall` support for Git repos)

### 🚀 Launch Templates
- Auto-generates launch files with support for:
  - Custom ROS arguments and node injection
  - Cross-package usage via `example/main.launch`

---

## ⚙️ How It Works

The tool reads a single configuration file `.robot_mindset_setup_assistant.yaml` that defines the project structure, interfaces, dependencies, and environment. It then uses **Jinja2 templates** to generate all required scaffolding.

> ❗️Note: Most generated files are intended to be **read-only** unless stated otherwise.

### Typical Workflow

1. **Initialize a project**
2. **Configure `.robot_mindset_setup_assistant.yaml`**
3. **Select, Generate one or multiple packages**
4. **Define interfaces & capabilities**:
   - Robots
   - Data processors
   - BT or PlanSys2 nodes
5. **Connect packages via composers**:
   - `robot_composer`: integrates tools, robots, simulation setups
   - `service_composer`: connects and chains processing components

---

## 📄 Example Configuration

```yaml
package:
  name: test_pkg
  description: A package to examine the test_pkg.
  version: "0.0.1"
  maintainer: Robot Mindset
  maintainer_email: robot.mindset@robot-mindset.com
  license:
    value: MIT
  language: cpp # python, cpp
  environment: ros.noetic # ros.noetic, ros.humble, (debian)
  dependencies:
    apt:
      - python3-catkin-tools
      - libspdlog-dev
    pip: []
    ros:
      - std_msgs
      - roscpp
    git: []
      # - name: sim2real_gap
      #   url: https://gitlab.cc-asp.fraunhofer.de/multirobot/strategy/sim2real_gap.git
      #   version: main
    packages: # CMakeLists.txt modifications
      - name: spdlog
        target_link: spdlog::spdlog

ros:
  interfaces:
    - name: "interface_topic_string_in"
      direction: "in"
      msgs: "std_msgs/String"
      type: "topic"
      description: "A ROS Topic interface."
```

---

## 📂 Package Structure

```
test_pkg/
├── test_pkg/
│   ├── doc/
│   │   └── index.md                    <-- add documention
│   ├── include/
│   │   ├── test_pkg                    <-- dependencies: pip
│   │   └── test_pkg.apt                <-- dependencies: apt
│   ├── src/
│   │   ├── ros/                        <-- add ROS code
│   │   ├── test_pkg/                   <-- add ROS code
│   │   └── node
│   ├── include/
│   │   ├── ros/                        <-- add ROS code
│   │   └── test_pkg/                   <-- add non-ROS code
│   ├── doc/
│   │   └── index.md
│   ├── launch/
│   │   └── test_pkg.launch
│   ├── example/
│   │   └── launch/
│   │       ├── main.launch
│   │       ├── single_serve.launch
│   │       ├── link_serve.launch
│   │       └── multi_serve.launch
│   ├── .rosinstall                     <-- dependencies: git repos
│   ├── CMakeLists.txt
│   └── package.xml                     <-- dependencies: rosdep
├── dev-setup/                          <-- git submodule
├── doxygen-awesome-css/                <-- git submodule
└── doxygen/
    ├── Doxyfile
    └── documentation/
```

---

## 💡 Tagging & Doc Conventions

While docstring conventions are not enforced yet, the following are recommended:

- `/// \brief` for class/function summaries
- `@param`, `@return`, `@ingroup` for grouping logical components
- Use Markdown-style headers in `doc/index.md` for the package overview

---

## 📌 Current Limitations

- Only ROS 1 or ROS 2 projects supported (not both in one workspace)
- `.msg`/`.srv` files must be created manually
- Automatic file edits discouraged unless noted as editable
- Only GitLab CI/CD is supported (for now)

---

## 📈 Roadmap

- [ ] Support non-ROS projects (pure Python/C++)
- [ ] NiceGUI-based interface
- [ ] Web-based project orchestration
- [ ] ROS 1 + ROS 2 hybrid support
- [ ] Multi-stage Docker build optimization
- [ ] Custom interface template injection

<Details>

Current state of the generated files with ROS2 support
- ROS2 Interfaces
  - Topics 
    - Header 
      - [ ] Ros_node.hpp 
      - [ ] Ros_input_interface.hpp 
      - [ ] Ros_output_interface.hpp 
      - [ ] Package_name.hpp 
      - [ ] Package_name_output_interface.hpp 
      - [ ] Package_name_input_interface.h 
    - Cpp 
      - [ ] Ros_node.cpp 
      - [ ] Package_name.cpp 
  - Serivce 
    - Header 
      - [ ] Ros_node.hpp 
      - [ ] Ros_input_interface.hpp 
      - [ ] Ros_output_interface.hpp 
      - [ ] Package_name.hpp 
      - [ ] Package_name_output_interface.hpp 
      - [ ] Package_name_input_interface.h 
    - Cpp 
      - [ ] Ros_node.cpp 
      - [ ] Package_name.cpp 
    - Actions 
      - Header 
        - [ ] Ros_node.hpp 
        - [ ] Ros_input_interface.hpp 
        - [ ] Ros_output_interface.hpp 
        - [ ] Package_name.hpp 
        - [ ] Package_name_output_interface.hpp 
        - [ ] Package_name_input_interface.h 
      - Cpp 
        - [ ] Ros_node.cpp 
        - [ ] Package_name.cpp

</Details>

---

## 📥 Installation & Usage

```bash
git clone https://github.com/robot-mindset/robot_mindset_setup_assistant.git
cd robot_mindset_setup_assistant
pip install -r requirements.txt

# Initialize your project
python robot_mindset_setup_assistant.py --config .robot_mindset_setup_assistant.yaml
```

---

## 📄 License

MIT License. See [LICENSE](LICENSE) for details.

---

## 🧑‍💻 Maintainer

**Robot Mindset**  
📧 robot.mindset@robot-mindset.com  
🌐 [robot-mindset.com](https://robot-mindset.com)
