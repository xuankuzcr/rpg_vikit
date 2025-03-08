#ifndef VIKIT_FILE_READER_H_
#define VIKIT_FILE_READER_H_

#include <fstream>
#include <vector>

namespace vk {

/**
 * Entry has to support the following operator
 *   std::istream& operator >>(std::istream&, Entry&);
 */
template <class Entry> class FileReader {
public:
  FileReader(const std::string &file)
      : hasEntry_(false), file_(file), file_stream_(file.c_str()) {}

  virtual ~FileReader() { file_stream_.close(); }

  void skip(int num_lines) {
    for (int idx = 0; idx < num_lines; ++idx) {
      if (!file_stream_.good())
        continue;
      file_stream_.ignore(1024, '\n');
      assert(file_stream_.gcount() < 1024);
    }
  }

  void skipComments() {
    while (file_stream_.good() && file_stream_.peek() == '#')
      skip(1);
  }

  /// Moves to the next entry in the file. Returns true, if there was a next
  /// entry, false otherwise.
  bool next() {
    if (file_stream_.good() && !file_stream_.eof()) {
      file_stream_ >> entry_;
      hasEntry_ = true;
      return true;
    }
    return false;
  }

  /// Read all entries at once.
  void readAllEntries(std::vector<Entry> &entries) {
    if (!hasEntry())
      next();
    do
      entries.push_back(entry());
    while (next());
  }

  /// Gets the current entry
  const Entry &entry() const { return entry_; }

  /// Determines whether the first entry was read
  const bool &hasEntry() const { return hasEntry_; }

private:
  bool hasEntry_;
  std::string file_;
  std::ifstream file_stream_;
  Entry entry_;
};

} // end namespace vk

#endif // VIKIT_FILE_READER_H_
