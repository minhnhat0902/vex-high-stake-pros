#include "csv.hpp"

#include <ctime>
#include <fstream>
#include <numeric>

CSV::CSV(const std::string& folder,
         const std::initializer_list<std::string> headers,
         const std::initializer_list<int> column_widths, int lines)
    : folder(folder),
      headers(headers),
      column_widths(column_widths),
      lines(lines) {
  // Set buffer size
  buffer.reserve(
      std::accumulate(  // Number of header characters
          headers.begin(), headers.end(), 0,
          [](int sum, const std::string& header) {
            return sum + header.size();
          }) +
      headers.size() +   // Number of comma and newline characters
      (std::accumulate(  // Number of data characters per line
           column_widths.begin(), column_widths.end(),
           0) +
       column_widths.size()  // Number of comma and newline characters
       ) * lines             // Number of lines
  );

  // Add headers to buffer
  for (const auto& header : headers) {
    buffer.append(header);
    buffer.append(",");
  }
  buffer.append(",\n");
}

void CSV::log(const std::vector<std::string>& data) {
  // Write to buffer
  for (const auto& datum : data) {
    buffer.append(datum);
    buffer.append(",");
  }
  buffer.append("\n");
}

CSV::~CSV() {
  save();  // Save the file
}

void CSV::save() {
  // Get current date and time.
  time_t now;
  time(&now);

  // Buffer for filename.
  char time_buffer[sizeof "2011-10-08T07:07:09Z"];
  strftime(time_buffer, sizeof time_buffer, "%FT%TZ", gmtime(&now));

  // Write buffer to a file named with the current date and time.
  std::string filename = "/usd/" + folder + "/" + time_buffer + ".csv";
  std::ofstream file(filename);
  file << buffer;
}
