# Robot Mindset Setup Assistant Agent

## Why this agent exists
- Automate ROS package bootstrapping with consistent conventions across teams.
- Orchestrate project prerequisites by reusing the `dev-setup` toolkit.
- Render project assets from Jinja2 templates so custom packages stay DRY.

## Core responsibilities
1. Read `.robot_mindset_setup_assistant.yaml` (or a variant) to understand the package description, interfaces, and dependencies.
2. Resolve lookup tables in `config/` to translate high-level options into concrete template variables.
3. Render the environment-specific Jinja2 template tree under `template/<environment>/` into the target workspace, keeping user-owned regions intact.
4. Enrich the newly generated project with optional modules (e.g., development setup, documentation) via composable helpers in `setup_assistant/`.

## Key modules
- `setup_assistant/create_pkg.py`: Entry-point for package generation. Loads configuration, renders templates, and preserves manual edits between `JINJA-BEGIN` / `JINJA-END` markers.
- `setup_assistant/dev_setup.py`: Adds the `dev-setup` submodule and invokes its bootstrapper (`python3 dev-setup/setup.py --environment ...`).
- `setup_assistant/doxygen_awesome.py`: Brings in the `doxygen-awesome-css` styling assets.
- `setup_assistant/package_versioning.py`: Thin wrapper around Git operations used to manage submodules and branches while generating output.
- `template/`: Environment-specific scaffolding (e.g., `ros/noetic`, `ros/humble`, `debian`). File and directory names may themselves be Jinja2 expressions.

## Generation flow
```
main.py → RosNoeticPackage → create_pkg.RosNoeticPackage.configure() → render_template_folder()
            └─ apply_dev_setup()        └─ preserve_user_content()
            └─ apply_doxygen_awesome()
```

1. **Bootstrap**: Choose the workspace destination and specific YAML configuration.
2. **Load context**: Merge base config with lookup table defaults (`config/ros/<distro>/lookup_tables/`).
3. **Render**: Walk the template tree, expanding every `.j2` file with Jinja2, including inline filters such as `regex_replace`.
4. **Protect edits**: Before overwriting an existing file, extract custom regions between `JINJA-BEGIN:<tag>` / `JINJA-END:<tag>` and reinsert them into the freshly rendered content.
5. **Post-process**: Optionally call `dev_setup` and `doxygen_awesome` helpers to clone auxiliary assets and run their installers.

## Configuration contracts
- **Package manifest**: `.robot_mindset_setup_assistant.yaml` (examples under `example/`). Defines package metadata, desired language, ROS environment, dependencies, and interface definitions.
- **Lookup tables**: YAML fragments in `config/` translate shorthand options (e.g., preset dependency bundles) into render-ready structures.
- **Spark hooks**: `sparks.yaml` is reserved for plug-in style extensions. Current file is a placeholder for future event-driven tasks.

## Working with templates
- Use Jinja2 expressions in file paths (`{{ args.package.name |to_snake_case}}`) and contents.
- Register new filters inside `create_pkg.py` if templates need additional transformations.
- Avoid manual edits in generated files unless wrapped in the `JINJA-BEGIN/JINJA-END` markers; otherwise they will be overwritten on the next run.
- Add new environments by creating a folder under `template/` and updating `get_template_folder()` to support the selector.
- **Template roots by environment**: ROS Noetic scaffolding lives under `robot_mindset_setup_assistant/template/ros/noetic/`; ROS Humble templates under `robot_mindset_setup_assistant/template/ros/humble/`; Debian packaging assets under `robot_mindset_setup_assistant/template/debian/`.

## Extending the agent
- **New module**: Add a helper in `setup_assistant/` and invoke it from the package class (`RosNoeticPackage`) once rendering is complete.
- **Custom workflows**: Subclass `GitFlowRepo` to change how submodules or feature branches are handled during generation.
- **Alternative entry point**: Mirror the pattern in `main.py` to target different configs or destination workspaces.

## Local usage pattern
```bash
pip install -r requirements.txt
python robot_mindset_setup_assistant/main.py
```
- Ensure the `dev-setup` submodule URL (configured in `setup_assistant/dev_setup.py`) resolves and the repository is accessible.
- Run inside a workspace with `git` available; submodule operations assume you have write access.

## Operational tips
- Enable verbose logging by configuring Loguru defaults before invoking the agent when debugging template issues.
- Keep template outputs under version control to spot diffs after regeneration.
- When adding dependencies, update both the configuration YAML and any supporting lookup tables to avoid mismatches during rendering.
