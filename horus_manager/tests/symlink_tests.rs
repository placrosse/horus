// Comprehensive Symlink Tests for HORUS Package Manager
// Tests symlink creation, detection, broken links, and runtime scenarios

use std::fs;
use std::path::{Path, PathBuf};
use tempfile::TempDir;

#[cfg(unix)]
use std::os::unix::fs::symlink;

// ============================================================================
// Test Setup Helpers
// ============================================================================

/// Create a test directory structure that simulates HORUS global cache
fn setup_global_cache(temp: &TempDir) -> PathBuf {
    let cache_dir = temp.path().join(".horus/cache");
    fs::create_dir_all(&cache_dir).unwrap();
    cache_dir
}

/// Create a test directory structure that simulates a local workspace
fn setup_local_workspace(temp: &TempDir) -> PathBuf {
    let workspace_dir = temp.path().join("workspace/.horus/packages");
    fs::create_dir_all(&workspace_dir).unwrap();
    workspace_dir
}

/// Create a mock package in the cache with metadata
fn create_mock_package(cache_dir: &Path, name: &str, version: &str) -> PathBuf {
    let pkg_dir = cache_dir.join(format!("{}@{}", name, version));
    fs::create_dir_all(&pkg_dir).unwrap();

    // Create metadata.json
    let metadata = serde_json::json!({
        "name": name,
        "version": version,
        "package_type": "HORUS"
    });
    fs::write(
        pkg_dir.join("metadata.json"),
        serde_json::to_string_pretty(&metadata).unwrap(),
    )
    .unwrap();

    // Create some content files
    fs::write(pkg_dir.join("lib.rs"), "// Mock library").unwrap();

    pkg_dir
}

/// Create a mock binary in the cache
fn create_mock_binary(cache_dir: &Path, name: &str) -> PathBuf {
    let bin_dir = cache_dir.join("bin");
    fs::create_dir_all(&bin_dir).unwrap();

    let bin_path = bin_dir.join(name);
    fs::write(&bin_path, "#!/bin/bash\necho 'mock binary'").unwrap();

    // Make executable on Unix
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut perms = fs::metadata(&bin_path).unwrap().permissions();
        perms.set_mode(0o755);
        fs::set_permissions(&bin_path, perms).unwrap();
    }

    bin_path
}

// ============================================================================
// Basic Symlink Creation Tests
// ============================================================================

#[cfg(unix)]
#[test]
fn test_symlink_creation_basic() {
    let temp = TempDir::new().unwrap();
    let source = temp.path().join("source_file");
    let link = temp.path().join("link_file");

    fs::write(&source, "test content").unwrap();
    symlink(&source, &link).unwrap();

    assert!(link.exists(), "Symlink should exist");
    assert!(link.is_symlink(), "Path should be a symlink");
    assert_eq!(
        fs::read_to_string(&link).unwrap(),
        "test content",
        "Content should be readable through symlink"
    );
}

#[cfg(unix)]
#[test]
fn test_symlink_creation_directory() {
    let temp = TempDir::new().unwrap();
    let source_dir = temp.path().join("source_dir");
    let link_dir = temp.path().join("link_dir");

    fs::create_dir_all(&source_dir).unwrap();
    fs::write(source_dir.join("file.txt"), "dir content").unwrap();

    symlink(&source_dir, &link_dir).unwrap();

    assert!(link_dir.exists(), "Directory symlink should exist");
    assert!(link_dir.is_symlink(), "Path should be a symlink");
    assert!(
        link_dir.join("file.txt").exists(),
        "File inside linked dir should be accessible"
    );
}

#[cfg(unix)]
#[test]
fn test_symlink_target_resolution() {
    let temp = TempDir::new().unwrap();
    let source = temp.path().join("real_file");
    let link = temp.path().join("symlink");

    fs::write(&source, "content").unwrap();
    symlink(&source, &link).unwrap();

    let resolved = fs::read_link(&link).unwrap();
    assert_eq!(resolved, source, "read_link should return original target");
}

// ============================================================================
// Broken Symlink Detection Tests
// ============================================================================

#[cfg(unix)]
#[test]
fn test_broken_symlink_detection() {
    let temp = TempDir::new().unwrap();
    let source = temp.path().join("will_be_deleted");
    let link = temp.path().join("broken_link");

    // Create source, symlink to it, then delete source
    fs::write(&source, "temporary").unwrap();
    symlink(&source, &link).unwrap();
    fs::remove_file(&source).unwrap();

    // The symlink path itself can be read
    assert!(
        link.read_link().is_ok(),
        "read_link should work on broken symlink"
    );

    // But the target doesn't exist
    assert!(
        !link.exists(),
        "exists() should return false for broken symlink"
    );

    // File type check still works
    let metadata = link.symlink_metadata().unwrap();
    assert!(
        metadata.file_type().is_symlink(),
        "symlink_metadata should detect symlink"
    );
}

#[cfg(unix)]
#[test]
fn test_broken_symlink_vs_nonexistent() {
    let temp = TempDir::new().unwrap();
    let broken_link = temp.path().join("broken");
    let nonexistent = temp.path().join("nonexistent");

    // Create a broken symlink
    let deleted_target = temp.path().join("deleted");
    fs::write(&deleted_target, "temp").unwrap();
    symlink(&deleted_target, &broken_link).unwrap();
    fs::remove_file(&deleted_target).unwrap();

    // Broken symlink: read_link succeeds, exists() fails
    assert!(
        broken_link.read_link().is_ok(),
        "Broken symlink should be readable"
    );
    assert!(!broken_link.exists(), "Broken symlink target doesn't exist");

    // Nonexistent path: read_link fails, exists() fails
    assert!(
        nonexistent.read_link().is_err(),
        "Nonexistent path can't be read as link"
    );
    assert!(!nonexistent.exists(), "Nonexistent path doesn't exist");
}

// ============================================================================
// Package Symlink Tests (simulating HORUS package manager behavior)
// ============================================================================

#[cfg(unix)]
#[test]
fn test_package_symlink_from_global_to_local() {
    let temp = TempDir::new().unwrap();
    let cache = setup_global_cache(&temp);
    let local = setup_local_workspace(&temp);

    // Create a package in global cache
    let pkg_cache = create_mock_package(&cache, "test_package", "1.0.0");

    // Simulate linking to local workspace
    let local_link = local.join("test_package");
    symlink(&pkg_cache, &local_link).unwrap();

    // Verify link works
    assert!(local_link.exists(), "Local link should exist");
    assert!(local_link.is_symlink(), "Should be a symlink");
    assert!(
        local_link.join("metadata.json").exists(),
        "Metadata should be accessible"
    );

    // Verify we can read metadata through the link
    let metadata_content = fs::read_to_string(local_link.join("metadata.json")).unwrap();
    assert!(
        metadata_content.contains("test_package"),
        "Metadata should contain package name"
    );
}

#[cfg(unix)]
#[test]
fn test_package_already_linked_detection() {
    let temp = TempDir::new().unwrap();
    let cache = setup_global_cache(&temp);
    let local = setup_local_workspace(&temp);

    let pkg_cache = create_mock_package(&cache, "my_pkg", "2.0.0");
    let local_link = local.join("my_pkg");

    // First link
    symlink(&pkg_cache, &local_link).unwrap();

    // Simulate "already linked" check (as done in run.rs)
    let already_linked = local_link.exists() || local_link.read_link().is_ok();
    assert!(already_linked, "Should detect existing symlink");
}

#[cfg(unix)]
#[test]
fn test_package_broken_link_after_cache_clean() {
    let temp = TempDir::new().unwrap();
    let cache = setup_global_cache(&temp);
    let local = setup_local_workspace(&temp);

    // Create package and link it
    let pkg_cache = create_mock_package(&cache, "will_clean", "1.0.0");
    let local_link = local.join("will_clean");
    symlink(&pkg_cache, &local_link).unwrap();

    // Verify link works
    assert!(local_link.exists(), "Link should work initially");

    // Simulate "horus clean" - delete the cached package
    fs::remove_dir_all(&pkg_cache).unwrap();

    // Now the link is broken
    assert!(
        !local_link.exists(),
        "Link target no longer exists after clean"
    );
    assert!(
        local_link.read_link().is_ok(),
        "Symlink itself still exists"
    );

    // This is the detection pattern used in monitor.rs
    let is_symlink = local_link
        .symlink_metadata()
        .map(|m| m.file_type().is_symlink())
        .unwrap_or(false);
    let symlink_broken = is_symlink && !local_link.exists();
    assert!(symlink_broken, "Should detect broken symlink");
}

// ============================================================================
// Binary Symlink Tests
// ============================================================================

#[cfg(unix)]
#[test]
fn test_binary_symlink_creation() {
    let temp = TempDir::new().unwrap();
    let cache = setup_global_cache(&temp);

    // Create a mock binary
    let bin_path = create_mock_binary(&cache, "horus-tool");

    // Create local bin directory
    let local_bin = temp.path().join(".horus/bin");
    fs::create_dir_all(&local_bin).unwrap();

    // Symlink the binary
    let local_bin_link = local_bin.join("horus-tool");
    symlink(&bin_path, &local_bin_link).unwrap();

    assert!(local_bin_link.exists(), "Binary symlink should exist");
    assert!(local_bin_link.is_symlink(), "Should be a symlink");

    // Verify it's executable
    use std::os::unix::fs::PermissionsExt;
    let perms = fs::metadata(&local_bin_link).unwrap().permissions();
    assert!(perms.mode() & 0o111 != 0, "Binary should be executable");
}

#[cfg(unix)]
#[test]
fn test_binary_symlink_overwrite_prevention() {
    let temp = TempDir::new().unwrap();
    let source1 = temp.path().join("bin1");
    let source2 = temp.path().join("bin2");
    let link = temp.path().join("bin_link");

    fs::write(&source1, "binary1").unwrap();
    fs::write(&source2, "binary2").unwrap();

    // Create first symlink
    symlink(&source1, &link).unwrap();

    // Attempting to create another symlink to the same path should fail
    let result = symlink(&source2, &link);
    assert!(result.is_err(), "Should not overwrite existing symlink");
}

// ============================================================================
// Version Mismatch Symlink Tests
// ============================================================================

#[cfg(unix)]
#[test]
fn test_symlink_version_mismatch_detection() {
    let temp = TempDir::new().unwrap();
    let cache = setup_global_cache(&temp);
    let local = setup_local_workspace(&temp);

    // Create package v1.0.0 and v2.0.0
    let pkg_v1 = create_mock_package(&cache, "versioned_pkg", "1.0.0");
    let _pkg_v2 = create_mock_package(&cache, "versioned_pkg", "2.0.0");

    // Link to v1
    let local_link = local.join("versioned_pkg");
    symlink(&pkg_v1, &local_link).unwrap();

    // Read the link target to detect version
    let target = fs::read_link(&local_link).unwrap();
    let target_str = target.to_string_lossy();

    assert!(
        target_str.contains("1.0.0"),
        "Link target should indicate version 1.0.0"
    );
    assert!(
        !target_str.contains("2.0.0"),
        "Link target should not be version 2.0.0"
    );
}

// ============================================================================
// Canonicalization Tests (for handling symlinks in workspace detection)
// ============================================================================

#[cfg(unix)]
#[test]
fn test_canonicalize_symlink_path() {
    let temp = TempDir::new().unwrap();
    let real_dir = temp.path().join("real_workspace");
    let link_dir = temp.path().join("linked_workspace");

    fs::create_dir_all(&real_dir).unwrap();
    symlink(&real_dir, &link_dir).unwrap();

    // Canonicalize should resolve the symlink
    let canonical = link_dir.canonicalize().unwrap();
    let real_canonical = real_dir.canonicalize().unwrap();

    assert_eq!(
        canonical, real_canonical,
        "Canonicalized paths should be equal"
    );
}

#[cfg(unix)]
#[test]
fn test_nested_symlinks_canonicalization() {
    let temp = TempDir::new().unwrap();
    let real = temp.path().join("real");
    let link1 = temp.path().join("link1");
    let link2 = temp.path().join("link2");

    fs::create_dir_all(&real).unwrap();
    symlink(&real, &link1).unwrap();
    symlink(&link1, &link2).unwrap(); // link2 -> link1 -> real

    let canonical = link2.canonicalize().unwrap();
    let real_canonical = real.canonicalize().unwrap();

    assert_eq!(canonical, real_canonical, "Should resolve nested symlinks");
}

// ============================================================================
// Cross-Platform Path Tests
// ============================================================================

#[test]
fn test_symlink_metadata_check() {
    let temp = TempDir::new().unwrap();
    let regular_file = temp.path().join("regular");
    let regular_dir = temp.path().join("dir");

    fs::write(&regular_file, "content").unwrap();
    fs::create_dir_all(&regular_dir).unwrap();

    // Regular file is not a symlink
    let file_meta = fs::symlink_metadata(&regular_file).unwrap();
    assert!(
        !file_meta.file_type().is_symlink(),
        "Regular file is not symlink"
    );

    // Regular directory is not a symlink
    let dir_meta = fs::symlink_metadata(&regular_dir).unwrap();
    assert!(
        !dir_meta.file_type().is_symlink(),
        "Regular directory is not symlink"
    );
}

// ============================================================================
// Runtime Integration Tests
// ============================================================================

#[cfg(unix)]
#[test]
fn test_full_package_install_flow() {
    let temp = TempDir::new().unwrap();
    let cache = setup_global_cache(&temp);
    let local = setup_local_workspace(&temp);

    // Step 1: Package is installed to global cache
    let pkg = create_mock_package(&cache, "full_test_pkg", "3.0.0");

    // Step 2: Check if local link exists (it shouldn't)
    let local_link = local.join("full_test_pkg");
    assert!(!local_link.exists(), "Link should not exist yet");
    assert!(
        local_link.read_link().is_err(),
        "read_link should fail for nonexistent"
    );

    // Step 3: Create symlink from local to global
    symlink(&pkg, &local_link).unwrap();

    // Step 4: Verify the package is accessible
    assert!(local_link.exists(), "Link should exist");
    assert!(
        local_link.join("metadata.json").exists(),
        "Metadata accessible"
    );
    assert!(local_link.join("lib.rs").exists(), "Lib file accessible");

    // Step 5: Verify we can read content
    let lib_content = fs::read_to_string(local_link.join("lib.rs")).unwrap();
    assert!(lib_content.contains("Mock library"), "Content readable");
}

#[cfg(unix)]
#[test]
fn test_multiple_packages_same_workspace() {
    let temp = TempDir::new().unwrap();
    let cache = setup_global_cache(&temp);
    let local = setup_local_workspace(&temp);

    // Install multiple packages
    let packages = vec![("pkg_a", "1.0.0"), ("pkg_b", "2.0.0"), ("pkg_c", "1.5.0")];

    for (name, version) in &packages {
        let pkg = create_mock_package(&cache, name, version);
        let local_link = local.join(name);
        symlink(&pkg, &local_link).unwrap();
    }

    // Verify all packages are accessible
    for (name, _) in &packages {
        let local_link = local.join(name);
        assert!(local_link.exists(), "Package {} should exist", name);
        assert!(
            local_link.is_symlink(),
            "Package {} should be symlink",
            name
        );
        assert!(
            local_link.join("metadata.json").exists(),
            "Package {} metadata should exist",
            name
        );
    }
}

#[cfg(unix)]
#[test]
fn test_relink_after_broken() {
    let temp = TempDir::new().unwrap();
    let cache = setup_global_cache(&temp);
    let local = setup_local_workspace(&temp);

    // Create and link package
    let pkg_v1 = create_mock_package(&cache, "relink_pkg", "1.0.0");
    let local_link = local.join("relink_pkg");
    symlink(&pkg_v1, &local_link).unwrap();

    // Simulate cache clean (delete cached package)
    fs::remove_dir_all(&pkg_v1).unwrap();
    assert!(!local_link.exists(), "Link should be broken");

    // Remove the broken symlink
    fs::remove_file(&local_link).unwrap();
    assert!(local_link.read_link().is_err(), "Symlink should be removed");

    // Re-install package (new version)
    let pkg_v2 = create_mock_package(&cache, "relink_pkg", "2.0.0");

    // Re-create symlink
    symlink(&pkg_v2, &local_link).unwrap();
    assert!(local_link.exists(), "New link should work");

    // Verify it points to v2
    let target = fs::read_link(&local_link).unwrap();
    assert!(
        target.to_string_lossy().contains("2.0.0"),
        "Should link to v2"
    );
}

// ============================================================================
// Error Handling Tests
// ============================================================================

#[cfg(unix)]
#[test]
fn test_symlink_to_nonexistent_target() {
    let temp = TempDir::new().unwrap();
    let nonexistent = temp.path().join("does_not_exist");
    let link = temp.path().join("link_to_nothing");

    // Creating a symlink to nonexistent path succeeds (this is valid)
    let result = symlink(&nonexistent, &link);
    assert!(
        result.is_ok(),
        "Symlink to nonexistent target is valid on Unix"
    );

    // But the link is "broken"
    assert!(!link.exists(), "Target doesn't exist");
    assert!(link.read_link().is_ok(), "Symlink itself exists");
}

#[cfg(unix)]
#[test]
fn test_symlink_permission_denied() {
    // Skip on root/elevated - can't test permission denied
    if unsafe { libc::geteuid() } == 0 {
        return;
    }

    let temp = TempDir::new().unwrap();
    let source = temp.path().join("source");
    fs::write(&source, "content").unwrap();

    // Create a directory we can't write to
    let protected_dir = temp.path().join("protected");
    fs::create_dir_all(&protected_dir).unwrap();

    use std::os::unix::fs::PermissionsExt;
    let mut perms = fs::metadata(&protected_dir).unwrap().permissions();
    perms.set_mode(0o555); // Read-only directory
    fs::set_permissions(&protected_dir, perms).unwrap();

    let link = protected_dir.join("link");
    let result = symlink(&source, &link);

    // Restore permissions for cleanup
    let mut perms = fs::metadata(&protected_dir).unwrap().permissions();
    perms.set_mode(0o755);
    fs::set_permissions(&protected_dir, perms).unwrap();

    assert!(
        result.is_err(),
        "Should fail to create symlink in protected dir"
    );
}

// ============================================================================
// Python Package Symlink Tests (horus_py special case)
// ============================================================================

#[cfg(unix)]
#[test]
fn test_python_package_lib_horus_symlink() {
    let temp = TempDir::new().unwrap();
    let cache = setup_global_cache(&temp);
    let local = setup_local_workspace(&temp);

    // Create horus_py package with lib/horus structure
    let pkg_dir = cache.join("horus_py@0.1.0");
    let lib_horus = pkg_dir.join("lib/horus");
    fs::create_dir_all(&lib_horus).unwrap();
    fs::write(lib_horus.join("__init__.py"), "# HORUS Python").unwrap();
    fs::write(lib_horus.join("core.py"), "# Core module").unwrap();

    // Create symlink named "horus" pointing to lib/horus
    let horus_link = local.join("horus");
    symlink(&lib_horus, &horus_link).unwrap();

    // Verify Python can import from this structure
    assert!(horus_link.exists(), "horus link should exist");
    assert!(
        horus_link.join("__init__.py").exists(),
        "__init__.py should be accessible"
    );
    assert!(
        horus_link.join("core.py").exists(),
        "core.py should be accessible"
    );
}

// ============================================================================
// Cleanup Tests
// ============================================================================

#[cfg(unix)]
#[test]
fn test_symlink_removal() {
    let temp = TempDir::new().unwrap();
    let source = temp.path().join("source");
    let link = temp.path().join("link");

    fs::write(&source, "content").unwrap();
    symlink(&source, &link).unwrap();

    // Remove symlink (not the target)
    fs::remove_file(&link).unwrap();

    assert!(!link.exists(), "Link should be removed");
    assert!(source.exists(), "Source should still exist");
}

#[cfg(unix)]
#[test]
fn test_directory_symlink_removal() {
    let temp = TempDir::new().unwrap();
    let source_dir = temp.path().join("source_dir");
    let link = temp.path().join("link");

    fs::create_dir_all(&source_dir).unwrap();
    fs::write(source_dir.join("file.txt"), "content").unwrap();
    symlink(&source_dir, &link).unwrap();

    // For directory symlinks, use remove_file not remove_dir
    fs::remove_file(&link).unwrap();

    assert!(!link.exists(), "Link should be removed");
    assert!(source_dir.exists(), "Source dir should still exist");
    assert!(
        source_dir.join("file.txt").exists(),
        "Files in source should remain"
    );
}
