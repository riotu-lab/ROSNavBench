"""
Utility functions for resolving relative and absolute paths in ROSNavBench configuration files.
"""
import os
from ament_index_python.packages import get_package_share_directory


def resolve_path(path, config_file_path=None, package_name='ROSNavBench'):
    """
    Resolve a path from configuration file.
    
    If the path is absolute (starts with /), it is returned as-is.
    If the path is relative, it is resolved relative to:
    1. The config file's directory (if config_file_path is provided)
    2. The package share directory (as fallback)
    
    Args:
        path: The path string from the config file
        config_file_path: Optional absolute path to the config file
        package_name: ROS package name (default: 'ROSNavBench')
    
    Returns:
        Resolved absolute path string
    """
    if not path or path == 'None':
        return path
    
    # If absolute path, return as-is
    if os.path.isabs(path):
        return path
    
    # If relative path, resolve it
    if config_file_path:
        # Resolve relative to config file directory
        config_dir = os.path.dirname(os.path.abspath(config_file_path))
        resolved = os.path.join(config_dir, path)
        # Normalize the path (resolve .. and .)
        resolved = os.path.normpath(resolved)
        # Convert to absolute
        resolved = os.path.abspath(resolved)
        return resolved
    else:
        # Fallback: resolve relative to package share directory
        try:
            package_dir = get_package_share_directory(package_name)
            resolved = os.path.join(package_dir, path)
            resolved = os.path.normpath(resolved)
            resolved = os.path.abspath(resolved)
            return resolved
        except Exception:
            # If package not found, try relative to current working directory
            return os.path.abspath(path)

def find_package_root(start_dir):
    """
    Walk upward to find the directory containing package.xml.
    """
    current = os.path.abspath(start_dir)
    while True:
        if os.path.isfile(os.path.join(current, 'package.xml')):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            return None
        current = parent


def resolve_paths_in_config(config_dict, config_file_path=None, package_name='ROSNavBench'):
    """
    Resolve all path fields in a configuration dictionary.
    
    Path fields that will be resolved:
    - world_path
    - map_path
    - map_png_path
    - models_path
    - nav_config
    - behaviour_tree_directory
    - urdf_file
    - model_file
    - results_directory
    
    Args:
        config_dict: Dictionary loaded from YAML config file
        config_file_path: Optional absolute path to the config file
        package_name: ROS package name (default: 'ROSNavBench')
    
    Returns:
        Dictionary with resolved paths (modifies in place and returns)
    """
    path_fields = [
        'world_path',
        'map_path',
        'map_png_path',
        'models_path',
        'nav_config',
        'behaviour_tree_directory',
        'urdf_file',
        'model_file',
        'results_directory'
    ]
    
    for field in path_fields:
        if field not in config_dict or not config_dict[field]:
            continue
        if field == 'models_path':
            # Support multiple model paths separated by ':'
            raw_paths = [p for p in config_dict[field].split(':') if p]
            resolved_paths = [
                resolve_path(path, config_file_path, package_name)
                for path in raw_paths
            ]
            config_dict[field] = ':'.join(resolved_paths)
            continue
        if field == 'results_directory' and config_file_path and not os.path.isabs(config_dict[field]):
            # Prefer package root for results if config is inside a package.
            config_dir = os.path.dirname(os.path.abspath(config_file_path))
            package_root = find_package_root(config_dir)
            if package_root:
                config_dict[field] = os.path.abspath(
                    os.path.normpath(os.path.join(package_root, config_dict[field]))
                )
                continue
        config_dict[field] = resolve_path(
            config_dict[field],
            config_file_path,
            package_name
        )
    
    return config_dict
