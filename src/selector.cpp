#include "selector.hpp"

Selector::Selector(pros::Controller controller, std::initializer_list<Key> keys)
    : controller(controller), keys(keys), key_index(0), is_clearing(false) {}

Key* Selector::get_current_key() { return &keys[key_index]; }

void Selector::update() {
  // Cycle through the keys with the controller left and right buttons.
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    key_index--;
    pros::delay(200);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    key_index++;
    pros::delay(200);
  }

  // Wrap around the keys.
  key_index = (key_index + keys.size()) % keys.size();

  Key* current_key = get_current_key();

  // Cycle through the values with the controller up and down buttons.
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
    current_key->index++;
    pros::delay(200);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    current_key->index--;
    pros::delay(200);
  }

  // Wrap around the values.
  current_key->index = (current_key->index + current_key->values.size()) %
                      current_key->values.size();
}

void Selector::display() {
  // Alternate between clearing and writing to the controller.
  is_clearing = !is_clearing;
  if (is_clearing) {
    controller.clear();
  }

  Key* current_key = get_current_key();

  // Display the current key on the controller.
  controller.print(0, 0, "%s: %s", current_key->name.c_str(),
                   current_key->values[current_key->index].c_str());
}
