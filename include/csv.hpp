/// @file csv.hpp
///
/// @brief Contains the CSV class for logging data to a CSV file.

#include "main.h"

class CSV {
  /// @brief The buffer for the CSV file.
  std::string buffer;

  /// @brief The folder for the CSV file.
  const std::string folder;

  /// @brief The headers for the CSV file.
  std::vector<std::string> headers;

  /// @brief The numbers of characters in each column.
  std::vector<int> column_widths;

  /// @brief The maximum number of lines in the CSV file.
  const int lines;

 public:
  /// @brief Construct a new CSV object.
  /// @param folder The folder for the CSV file.
  /// @param headers The column headers for the CSV file.
  /// @param column_widths The number of characters in each column of the CSV
  /// file.
  /// @param lines The maximum number of lines in the CSV file.
  ///
  /// The csv file will be saved with the current date and time as the filename.
  ///
  /// Example:
  ///
  /// ```
  /// CSV logger(
  ///   "path/to/folder", // The folder for the CSV file
  ///   {"X", "Y", "θ"},  // Column headers are X, Y and θ
  ///   {6, 6, 6},        // Each column takes 6 characters
  ///   1000              // The CSV file has a maximum of 1000 lines
  /// );
  /// ```
  CSV(const std::string& folder,
             const std::initializer_list<std::string>& headers = {},
             const std::initializer_list<int>& column_widths = {},
             int lines);

  /// @brief Destroy the CSV object.
  ~CSV();

  /// @brief Log data to the CSV file.
  /// @param data The data to log.
  void log(const std::vector<std::string>& data);

  /// @brief Save the CSV file.
  void save();
};
