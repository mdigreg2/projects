#!/usr/bin/env bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
TARGET="$SCRIPT_DIR/../.git/hooks/pre-commit"
LOCAL="$SCRIPT_DIR/hooks/org_precommit.sh"

# tput color-setting strings
YEL=`tput setaf 3`
GRE=`tput setaf 2`
CYN=`tput setaf 6`
RST=`tput sgr0`

# If target file is already a symlink, delete
if [ -L "$TARGET" ]
then

    # If existing link points to source, return
    if [ "$(readlink -f "$TARGET")" == "$LOCAL" ]
    then
	printf "${GRE}Existing symlink kept: %-20s %s ${RST}\n" "${2##/*/}" "($TARGET)"
	exit 1
    else
	rm "$TARGET"
	printf "${YEL}Removed symlink %-20s %s ${RST}\n" "${2##/*/}" "($TARGET)"
    fi

# If target exists and is not a symlink, backup
elif [ -e "$TARGET" ]
then
    until [ ! -e "$TARGET.old${NUM}" ]
    do
	NUM=$((NUM+1))
    done
    mv "$TARGET" "$TARGET.old${NUM}"
    printf "${YEL}Moved %-15s to %-15s (%-30s to %s.old)${RST}\n" "${2##/*/}" "${2##/*/}.old${NUM}" "$TARGET" "$TARGET.old${NUM}"
fi

ln -s "$LOCAL" "$TARGET"
printf "${CYN}Created symlink %-20s (%-50s -> %s) ${RST}\n" "${2##/*/}" "$LOCAL" "$TARGET"
