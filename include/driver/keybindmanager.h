#include <vex.h>
#include <vector>
#include <functional>
#include "util.h"
#include "config.h"

#pragma once

class KeybindCondition {
    private:
        enum ConditionType {
            KEY_CONDITION,
            CUSTOM_CONDITION
        };

        ConditionType type;
        controller::button key;
        bool state;
        std::function<bool()> customCondition;

    public:
        // Constructor for key conditions
        KeybindCondition(controller::button k, bool s) : type(KEY_CONDITION), key(k), state(s) {}

        // Constructor for custom conditions
        KeybindCondition(std::function<bool()> condition) : type(CUSTOM_CONDITION), customCondition(condition) {}

        bool isMet() const {
            switch (type) {
                case KEY_CONDITION:
                    return key.pressing() == state;
                case CUSTOM_CONDITION:
                    return customCondition();
                default:
                    return false;
            }
        }
};

class KeyBinding {
    friend class KeybindManager;

    private:
        std::function<void()> onPressedCallback;
        std::function<void()> onReleasedCallback;
        std::function<void()> onPressingCallback;

        std::vector<KeybindCondition> conditions;
        string name = "";
        bool lastState = false;

        void poll() {
            if (name == "") {
                throw std::runtime_error("Keybinding name not set");
            }

            bool currentState = pressed();
            if (currentState && !lastState) {
                if (onPressedCallback) {
                    onPressedCallback();
                }
            } else if (!currentState && lastState) {
                if (onReleasedCallback) {
                    onReleasedCallback();
                }
            } else if (currentState && onPressingCallback) {
                onPressingCallback();
            }
            lastState = currentState;
        }

    public:
        KeyBinding& addCondition(controller::button key, bool state) {
            conditions.emplace_back(key, state);
            return *this;
        }

        // Add a condition with a custom function
        KeyBinding& addCondition(std::function<bool()> condition) {
            conditions.emplace_back(condition);
            return *this;
        }

        // Add a condition
        KeyBinding& addCondition(KeybindCondition condition) {
            conditions.push_back(condition);
            return *this;
        }

        bool pressed() const {
            for (const auto& condition : conditions) {
                if (!condition.isMet()) {
                    return false;
                }
            }
            return true;
        }

        KeyBinding& onPressed(std::function<void()> onPressed) {
            onPressedCallback = onPressed;
            return *this;
        }

        KeyBinding& onReleased(std::function<void()> onReleased) {
            onReleasedCallback = onReleased;
            return *this;
        }

        KeyBinding& onPressing(std::function<void()> onPressing) {
            onPressingCallback = onPressing;
            return *this;
        }

        KeyBinding& setName(string n) {
            name = n;
            return *this;
        }
};

class KeybindManager {
    friend class KeyBinding;

    private:
        std::vector<KeyBinding> keybinds;

        int internalKeybindTask(){
            while (true) {
                for (auto& keybind : keybinds) {
                    keybind.poll();
                }
                task::sleep(10);
            }
            return 0;
        }

        static int internalKeybindTaskWrapper(void* keybindManagerInstance) {
            return static_cast<KeybindManager*>(keybindManagerInstance)->internalKeybindTask();
        }

    public:
        KeybindManager() {
            task t(internalKeybindTaskWrapper, this);
        }

        KeybindManager& registerKeybinding(KeyBinding keybind) {
            if (keybind.name == "") {
                throw std::runtime_error("Keybinding name not set");
            }
            
            // if the name is already taken, replace it
            for (auto& k : keybinds) {
                if (k.name == keybind.name) {
                    k = keybind;
                    return *this;
                }
            }

            keybinds.push_back(keybind);
            return *this;
        }
};


static KeybindManager keybindManager = KeybindManager();