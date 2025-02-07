/// @file selector.hpp
///
/// @brief Contains the Selector class for selector on the controller.

#include "main.h"

/// @brief A list of values for each key of the selector.
struct Key {
  /// @brief The name of the key.
  std::string name;
  /// @brief The list of values for the key.
  std::vector<std::string> values;
  /// @brief The current index of the key.
  int index;
};

class Selector {
  /// @brief The controller object.
  pros::Controller controller;

  /// @brief The list of keys for the selector.
  std::vector<Key> keys;

  /// @brief The index of the current selected key.
  int key_index;

 public:
  /// @brief Constructor for the Selector class.
  /// @param controller The controller object to use for the selector.
  /// @param keys The list of keys for the selector.
  Selector(pros::v5::Controller controller, std::initializer_list<Key> keys);

  /// @brief Returns the current selected key.
  /// @return The current selected key.
  Key get_current_key();

  /// @brief Updates the selector and returns the current selected key.
  void update();

  /// @brief Displays the selector on the controller.
  void display();
};
