# Environment Customization with Ansible

This guide demonstrates how to extend and customize the AutoSDV development environment using Ansible automation.

## Provisioning Architecture

AutoSDV employs Ansible for automated environment provisioning, managed through the `setup-dev-env.sh` script. The configuration management system is organized in `scripts/setup-dev-env/ansible/` with a modular, role-based architecture.

**Key Benefits:**
- **Modularity**: Each role handles a specific component (ZED SDK, network configuration, etc.)
- **Maintainability**: Clean separation of concerns across different system components
- **Reusability**: Roles can be easily shared and reused across different environments

## Creating Custom Roles

Extending the AutoSDV environment is straightforward with Ansible's role-based system. This walkthrough demonstrates adding a new software component through a custom role.

### Step 1: Role Structure Setup

Create a new role directory with descriptive naming:

```bash
# Create role directory structure
mkdir -p scripts/setup-dev-env/ansible/roles/my_driver/tasks
mkdir -p scripts/setup-dev-env/ansible/roles/my_driver/defaults
```

**Naming Convention**: Use descriptive names like `my_driver`, `core_library`, or `sensor_integration`

### Step 2: Task Definition

Define the installation and configuration steps in `tasks/main.yml`:

```yaml
# scripts/setup-dev-env/ansible/roles/my_driver/tasks/main.yml
---
- name: "Download {{ my_driver_name }} package"
  ansible.builtin.get_url:
    url: "{{ my_driver_deb_url }}"
    dest: "/tmp/{{ my_driver_deb_filename }}"
    mode: '0644'
  tags: ['download', 'my_driver']

- name: "Install {{ my_driver_name }} package"
  ansible.builtin.apt:
    deb: "/tmp/{{ my_driver_deb_filename }}"
  become: yes
  tags: ['install', 'my_driver']

- name: "Clean up temporary files"
  ansible.builtin.file:
    path: "/tmp/{{ my_driver_deb_filename }}"
    state: absent
  tags: ['cleanup', 'my_driver']
```

**Best Practices:**
- Use descriptive task names with variable interpolation
- Add appropriate tags for selective execution
- Include cleanup tasks to maintain system hygiene

### Step 3: Variable Configuration

Parameterize your role through default variables for flexibility and maintainability:

```yaml
# scripts/setup-dev-env/ansible/roles/my_driver/defaults/main.yml
---
# Package information
my_driver_name: "My Awesome Driver"
my_driver_version: "1.2.3"
my_driver_description: "High-performance driver for autonomous vehicles"

# Download configuration
my_driver_deb_filename: "my-driver_{{ my_driver_version }}_{{ ansible_architecture }}.deb"
my_driver_deb_url: "https://releases.example.com/{{ my_driver_version }}/{{ my_driver_deb_filename }}"
my_driver_checksum: "sha256:abc123..."  # Optional but recommended

# Installation options
my_driver_force_install: false
my_driver_enable_service: true
```

**Configuration Benefits:**
- **Flexibility**: Easy customization without modifying task files
- **Version Management**: Centralized version control
- **Architecture Support**: Automatic architecture detection
- **Security**: Checksum verification for package integrity

### Step 4: Role Integration

Integrate your role into the main provisioning playbook:

```yaml
# scripts/setup-dev-env/ansible/playbooks/autosdv.yaml
- hosts: all
  become: yes
  roles:
    # Core system roles
    - role: base_system
    - role: docker_setup
    
    # Hardware-specific roles
    - role: nvidia_drivers
    - role: my_driver
      when: install_my_driver | default(true)
    
    # Application roles
    - role: autoware_setup
```

**Integration Options:**
- **Conditional Installation**: Use `when` clauses for optional components
- **Role Dependencies**: Define prerequisites in `meta/main.yml`
- **Variable Overrides**: Customize behavior through playbook variables

## Development Best Practices

### Code Quality Standards

**Modularity**
- Keep roles focused on single, well-defined components
- Avoid mixing concerns within a single role
- Create role dependencies when logical relationships exist

**Idempotency**
- Ensure tasks can be executed multiple times safely
- Use Ansible's built-in modules for automatic idempotency
- Test role execution in both clean and pre-configured environments

**Parameterization**
- Abstract all configurable values into `defaults/main.yml`
- Use descriptive variable names with consistent prefixes
- Provide sensible defaults while allowing easy customization

**Documentation**
- Include a `README.md` for complex roles
- Document variable purposes and acceptable values
- Provide usage examples and common configuration scenarios

### Testing and Validation

```bash
# Test individual roles
ansible-playbook --tags "my_driver" playbooks/autosdv.yaml

# Validate syntax
ansible-playbook --syntax-check playbooks/autosdv.yaml

# Dry run execution
ansible-playbook --check playbooks/autosdv.yaml
```

### Community Guidelines

Following these practices ensures AutoSDV remains:
- **Robust**: Reliable operation across different environments
- **Maintainable**: Easy to update and extend over time
- **Accessible**: Simple for new contributors to understand and modify

Contributions that adhere to these standards help maintain the high quality and usability of the AutoSDV development environment.