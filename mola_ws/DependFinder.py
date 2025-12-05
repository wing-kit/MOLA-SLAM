import os
import xml.etree.ElementTree as ET
import sys
import subprocess
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

def get_ros_distro():
    """ดึงชื่อ ROS distro ที่ใช้งานอยู่"""
    return os.environ.get('ROS_DISTRO', 'humble')

def get_installed_version(package_name):
    """
    หาเวอร์ชันที่ติดตั้งจริงในเครื่อง
    1. ลองหาแบบ ROS Package (ament)
    2. ลองหาแบบ System Package (dpkg) - แบบชื่อเดิม
    3. ลองหาแบบ System Package (dpkg) - แบบมี prefix ros-distro
    """
    # --- 1. Try as ROS package via ament_index ---
    try:
        share_dir = get_package_share_directory(package_name)
        package_xml_path = os.path.join(share_dir, 'package.xml')
        if os.path.exists(package_xml_path):
            try:
                tree = ET.parse(package_xml_path)
                root = tree.getroot()
                ver = root.find('version')
                if ver is not None:
                    return f"{ver.text} (ROS)"
            except:
                pass
    except (PackageNotFoundError, ValueError):
        # ValueError occurs when package_name is not a valid ROS package name (e.g., system packages like libusb-1.0-dev)
        pass

    # --- 2. Try as System package with original name ---
    try:
        cmd = ['dpkg-query', '-W', '-f=${Version}', package_name]
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode == 0 and result.stdout.strip():
            return f"{result.stdout.strip()} (Apt)"
    except:
        pass

    # --- 3. Try as ROS System package (ros-distro-package) ---
    distro = get_ros_distro()
    # แปลง underscore เป็น dash สำหรับชื่อ apt package
    apt_package_name = f"ros-{distro}-{package_name.replace('_', '-')}"
    try:
        cmd = ['dpkg-query', '-W', '-f=${Version}', apt_package_name]
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode == 0 and result.stdout.strip():
            return f"{result.stdout.strip()} (Apt: {apt_package_name})"
    except:
        pass

    return "❌ Not Found"

def get_deps_and_check_versions(workspace_path):
    local_packages = set()
    external_dependencies = set()
    
    depend_tags = ['build_depend', 'build_export_depend', 'buildtool_depend', 
                   'buildtool_export_depend', 'exec_depend', 'depend', 'test_depend']

    print(f"[*] Scanning workspace: {workspace_path}")
    print(f"[*] ROS Distro: {get_ros_distro()}\n")

    # หา Local Packages
    for root, dirs, files in os.walk(workspace_path):
        if 'package.xml' in files:
            try:
                tree = ET.parse(os.path.join(root, 'package.xml'))
                name = tree.getroot().find('name').text
                if name: 
                    local_packages.add(name.strip())
            except: 
                pass

    print(f"[*] Found {len(local_packages)} local packages\n")

    # หา External Dependencies
    for root, dirs, files in os.walk(workspace_path):
        if 'package.xml' in files:
            try:
                tree = ET.parse(os.path.join(root, 'package.xml'))
                
                for tag in depend_tags:
                    for dep in tree.getroot().findall(tag):
                        if dep.text:
                            dep_name = dep.text.strip()
                            if dep_name not in local_packages:
                                external_dependencies.add(dep_name)
            except: 
                pass

    return external_dependencies

if __name__ == "__main__":
    target_path = sys.argv[1] if len(sys.argv) > 1 else "."
    
    deps_set = get_deps_and_check_versions(target_path)
    
    print("=" * 80)
    print(f"{'Dependency':<40} | {'Installed Version':<35}")
    print("=" * 80)
    
    if not deps_set:
        print("No external dependencies found.")
    else:
        not_found_count = 0
        # Sort dependencies by name
        for dep_name in sorted(deps_set):
            version = get_installed_version(dep_name)
            print(f"{dep_name:<40} | {version:<35}")
            
            if "Not Found" in version:
                not_found_count += 1
            
    print("=" * 80)
    print(f"Total dependencies: {len(deps_set)}")
    if not_found_count > 0:
        print(f"⚠️  Missing: {not_found_count} packages")
        print(f"\n💡 To install missing dependencies, run:")
        print(f"   cd {target_path}")
        print(f"   rosdep install --from-paths src --ignore-src -r -y")
    print("=" * 80)