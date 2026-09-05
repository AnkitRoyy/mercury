v4l2-ctl --list-devices | awk '/EMEET/{getline; print}' | xargs && v4l2-ctl --list-devices | awk '/Angetube/{getline; print}' | xargs
