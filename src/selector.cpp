#include "selector.hpp"

Selector::Selector(pros::Controller controller, std::initializer_list<Key> keys)
    : controller(controller), keys(keys), key_index(0) {}

Key* Selector::get_current_key() { return &keys[key_index]; }

void Selector::update() {
  // Cycle through the keys with the controller left and right buttons.
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    key_index--;
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    key_index++;
  }

  // Wrap around the keys.
  key_index = (key_index + keys.size()) % keys.size();

  Key* current_key = get_current_key();

  // Cycle through the values with the controller up and down buttons.
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
    current_key->index++;
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    current_key->index--;
  }

  // Wrap around the values.
  current_key->index = (current_key->index + current_key->values.size()) %
                      current_key->values.size();
}

void Selector::display() {
  Key* current_key = get_current_key();

  // Display the current key on the controller.
  controller.print(0, 0, "%s: %s", current_key->name.c_str(),
                   current_key->values[current_key->index].c_str());
}
