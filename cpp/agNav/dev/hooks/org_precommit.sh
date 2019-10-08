#!/usr/bin/env bash

ORG_FILE="docs/index.org"
HTML_FILE="docs/index.html"
FILES=`git diff --cached --name-status`

if [[ $FILES =~ $ORG_FILE ]]
then
    if [ -x "$(command -v emacs)" ]
    then
	EMACSVERS=$(emacs --version | head -n 1 | cut -d ' ' -f3 | cut -d '.' -f1)
	if (( $EMACSVERS < 24 ))
	then
	    echo "Warning: Emacs version $EMACSVERS untested"
	fi
	ORG_EXPORT="org-html-export-to-html"

    	echo "Emacs: compiling $ORG_FILE into $HTML_FILE"
	emacs --no-site-lisp\
	      --batch\
	      --eval="(setq vc-handled-backends ())"\
	      --eval="(add-to-list 'load-path \"./dev/helpers\")"\
	      --eval="(require 'htmlize)"\
	      $ORG_FILE -f $ORG_EXPORT --kill
	EMACSEXIT=$?
	if ! [ $EMACSEXIT -eq 0 ]
	then
	    echo "Command failed: $(history 2 | head -n1 | cut -d ' ' -f4-)"
	    echo "Emacs error code $EMACSEXIT"
	    echo "Changes not committed"
	    exit 1
	else
	    echo "Emacs: $HTML_FILE written successfully"
	fi
	echo "Git: adding $HTML_FILE to commit"
	git add --force $HTML_FILE
	GITEXIT=$?
	if ! [ $GITEXIT -eq 0 ]
	then
	    echo "Command failed: $GITADD"
	    echo "Git error code $(history 2 | head -n1 | cut -d ' ' -f4-)"
	    echo "Changes not committed"
	    exit 1
	else
	    echo "Git: $HTML_FILE staged for commit"
	fi
    else
	echo "Warning: Emacs not installed, $HTML_FILE not committed"
	exit 1
    fi
fi
