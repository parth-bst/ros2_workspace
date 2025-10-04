#!/bin/bash

# Dotfiles Management Script for Pi
# This script manages dotfiles by syncing between the Pi's home directory and git-tracked copies

set -e  # Exit on any error

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
DOTFILES_DIR="$WORKSPACE_DIR/dotfiles"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to sync dotfile from Pi to git repo
sync_to_git() {
    local pi_file="$1"
    local git_file="$2"
    local filename="$(basename "$pi_file")"
    
    print_status "Syncing $filename from Pi to git repo..."
    
    if [ -f "$pi_file" ]; then
        # Create backup of current git file
        if [ -f "$git_file" ]; then
            cp "$git_file" "${git_file}.backup.$(date +%Y%m%d_%H%M%S)"
        fi
        
        # Copy from Pi to git repo
        cp "$pi_file" "$git_file"
        print_success "$filename synced to git repo"
    else
        print_warning "$pi_file not found on Pi"
    fi
}

# Function to sync dotfile from git repo to Pi
sync_to_pi() {
    local git_file="$1"
    local pi_file="$2"
    local filename="$(basename "$pi_file")"
    
    print_status "Syncing $filename from git repo to Pi..."
    
    if [ -f "$git_file" ]; then
        # Create backup of current Pi file
        if [ -f "$pi_file" ]; then
            cp "$pi_file" "${pi_file}.backup.$(date +%Y%m%d_%H%M%S)"
        fi
        
        # Copy from git repo to Pi
        cp "$git_file" "$pi_file"
        print_success "$filename synced to Pi"
    else
        print_warning "$git_file not found in git repo"
    fi
}

# Function to show differences between files
show_diff() {
    local pi_file="$1"
    local git_file="$2"
    local filename="$(basename "$pi_file")"
    
    if [ -f "$pi_file" ] && [ -f "$git_file" ]; then
        if ! diff -q "$pi_file" "$git_file" > /dev/null 2>&1; then
            print_warning "Differences found in $filename:"
            diff -u "$git_file" "$pi_file" || true
            echo ""
        else
            print_success "$filename files are identical"
        fi
    else
        print_warning "Cannot compare $filename - one or both files missing"
    fi
}

# Main dotfile mappings
declare -A DOTFILE_MAPPINGS=(
    ["$HOME/.bashrc"]="$DOTFILES_DIR/.bashrc_pi"
    ["$HOME/.bash_aliases"]="$DOTFILES_DIR/.bash_aliases_pi"
    ["$HOME/.vimrc"]="$DOTFILES_DIR/.vimrc_pi"
    ["$HOME/.gitconfig"]="$DOTFILES_DIR/.gitconfig_pi"
    ["$HOME/.ssh/config"]="$DOTFILES_DIR/ssh_config_pi"
    ["$HOME/.profile"]="$DOTFILES_DIR/.profile_pi"
    ["$HOME/.bash_profile"]="$DOTFILES_DIR/.bash_profile_pi"
)

# Package list files
APT_PACKAGES_FILE="$DOTFILES_DIR/apt_packages_pi.txt"
PIP_PACKAGES_FILE="$DOTFILES_DIR/pip_packages_pi.txt"
PIP_USER_PACKAGES_FILE="$DOTFILES_DIR/pip_user_packages_pi.txt"

# Function to backup apt packages
backup_apt_packages() {
    print_status "Backing up apt packages..."
    
    # Get manually installed packages (exclude auto-installed dependencies)
    apt-mark showmanual > "$APT_PACKAGES_FILE"
    
    # Also get a list of all installed packages for reference
    dpkg --get-selections > "${APT_PACKAGES_FILE}.all"
    
    print_success "Apt packages backed up to $(basename "$APT_PACKAGES_FILE")"
}

# Function to restore apt packages
restore_apt_packages() {
    print_status "Restoring apt packages..."
    
    if [ -f "$APT_PACKAGES_FILE" ]; then
        print_status "Installing packages from $(basename "$APT_PACKAGES_FILE")..."
        sudo apt update
        xargs -a "$APT_PACKAGES_FILE" sudo apt install -y
        print_success "Apt packages restored"
    else
        print_warning "Apt packages file not found: $(basename "$APT_PACKAGES_FILE")"
    fi
}

# Function to backup pip packages
backup_pip_packages() {
    print_status "Backing up pip packages..."
    
    # Backup system-wide pip packages
    pip list --format=freeze > "$PIP_PACKAGES_FILE"
    
    # Backup user-installed pip packages (--user flag)
    pip list --user --format=freeze > "$PIP_USER_PACKAGES_FILE"
    
    print_success "Pip packages backed up to $(basename "$PIP_PACKAGES_FILE") and $(basename "$PIP_USER_PACKAGES_FILE")"
}

# Function to restore pip packages
restore_pip_packages() {
    print_status "Restoring pip packages..."
    
    if [ -f "$PIP_PACKAGES_FILE" ]; then
        print_status "Installing system-wide pip packages..."
        pip install -r "$PIP_PACKAGES_FILE"
    else
        print_warning "System pip packages file not found: $(basename "$PIP_PACKAGES_FILE")"
    fi
    
    if [ -f "$PIP_USER_PACKAGES_FILE" ]; then
        print_status "Installing user pip packages..."
        pip install --user -r "$PIP_USER_PACKAGES_FILE"
    else
        print_warning "User pip packages file not found: $(basename "$PIP_USER_PACKAGES_FILE")"
    fi
    
    print_success "Pip packages restored"
}

# Function to show package status
show_package_status() {
    print_status "Package management status:"
    echo ""
    
    # Check apt packages
    if [ -f "$APT_PACKAGES_FILE" ]; then
        local apt_count=$(wc -l < "$APT_PACKAGES_FILE")
        echo "  Apt packages: $apt_count packages tracked"
    else
        echo "  Apt packages: Not backed up"
    fi
    
    # Check pip packages
    if [ -f "$PIP_PACKAGES_FILE" ]; then
        local pip_count=$(wc -l < "$PIP_PACKAGES_FILE")
        echo "  System pip packages: $pip_count packages tracked"
    else
        echo "  System pip packages: Not backed up"
    fi
    
    if [ -f "$PIP_USER_PACKAGES_FILE" ]; then
        local pip_user_count=$(wc -l < "$PIP_USER_PACKAGES_FILE")
        echo "  User pip packages: $pip_user_count packages tracked"
    else
        echo "  User pip packages: Not backed up"
    fi
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  sync-to-git    - Sync all dotfiles from Pi to git repo"
    echo "  sync-to-pi     - Sync all dotfiles from git repo to Pi"
    echo "  diff           - Show differences between Pi and git repo files"
    echo "  status         - Show status of all tracked dotfiles"
    echo "  backup-packages - Backup apt and pip packages"
    echo "  restore-packages - Restore apt and pip packages"
    echo "  package-status - Show package backup status"
    echo "  help           - Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 sync-to-git        # Backup current Pi configs to git"
    echo "  $0 sync-to-pi         # Restore Pi configs from git"
    echo "  $0 backup-packages    # Backup all installed packages"
    echo "  $0 restore-packages   # Restore all packages on new Pi"
    echo "  $0 diff               # See what's different"
}

# Function to sync all dotfiles to git
sync_all_to_git() {
    print_status "Syncing all dotfiles from Pi to git repo..."
    echo ""
    
    for pi_file in "${!DOTFILE_MAPPINGS[@]}"; do
        git_file="${DOTFILE_MAPPINGS[$pi_file]}"
        sync_to_git "$pi_file" "$git_file"
    done
    
    echo ""
    print_success "All dotfiles synced to git repo!"
}

# Function to sync all dotfiles to Pi
sync_all_to_pi() {
    print_status "Syncing all dotfiles from git repo to Pi..."
    echo ""
    
    for pi_file in "${!DOTFILE_MAPPINGS[@]}"; do
        git_file="${DOTFILE_MAPPINGS[$pi_file]}"
        sync_to_pi "$git_file" "$pi_file"
    done
    
    echo ""
    print_success "All dotfiles synced to Pi!"
}

# Function to show differences
show_all_diffs() {
    print_status "Checking differences between Pi and git repo files..."
    echo ""
    
    for pi_file in "${!DOTFILE_MAPPINGS[@]}"; do
        git_file="${DOTFILE_MAPPINGS[$pi_file]}"
        show_diff "$pi_file" "$git_file"
    done
}

# Function to show status
show_status() {
    print_status "Dotfiles status:"
    echo ""
    
    for pi_file in "${!DOTFILE_MAPPINGS[@]}"; do
        git_file="${DOTFILE_MAPPINGS[$pi_file]}"
        filename="$(basename "$pi_file")"
        
        pi_exists=""
        git_exists=""
        
        [ -f "$pi_file" ] && pi_exists="✓" || pi_exists="✗"
        [ -f "$git_file" ] && git_exists="✓" || git_exists="✗"
        
        echo "  $filename: Pi[$pi_exists] Git[$git_exists]"
    done
}

# Main script logic
case "${1:-help}" in
    "sync-to-git")
        sync_all_to_git
        ;;
    "sync-to-pi")
        sync_all_to_pi
        ;;
    "diff")
        show_all_diffs
        ;;
    "status")
        show_status
        ;;
    "backup-packages")
        backup_apt_packages
        backup_pip_packages
        ;;
    "restore-packages")
        restore_apt_packages
        restore_pip_packages
        ;;
    "package-status")
        show_package_status
        ;;
    "help"|*)
        show_usage
        ;;
esac
