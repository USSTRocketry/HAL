Unified Storage Design
(chatgpt assisted)

---

## Interfaces

### IWriter
- Minimal write-only interface
- Can optionally have Flush() for buffered backends

struct IWriter {
    virtual ~IWriter() = default;
    virtual Result Write(std::span<std::byte> data) = 0;
    virtual Result Flush() { return Result::Success; } // optional no-op
};

### IRawStorage
- Implements IWriter for raw backends (FRAM, EEPROM)
- Optional Read() if sequential reading is supported

struct IRawStorage : IWriter {
    virtual Result Read(std::span<std::byte> buffer) { return Result::Fail; } // optional
};

### IFile
- Implements IWriter (and optionally Read)
- Adds file-specific operations

struct IFile : IWriter {
    virtual Result Read(std::span<std::byte> buffer) { return Result::Fail; }
    virtual Result Seek(size_t pos) = 0;
    virtual size_t Tell() = 0;
    virtual void Close() = 0;
};

### IFileSystem
- Manages multiple files, only for filesystem-capable storage (SD)

struct IFileSystem {
    virtual IFile* OpenFile(const char* path) = 0;
    virtual Result RemoveFile(const char* path) = 0;
    virtual bool Exists(const char* path) = 0;
    virtual std::vector<std::string> ListFiles() = 0;
};

---

## Relationships Diagram

                        +----------------------+
                        |      IWriter         |
                        |----------------------|
                        | +Write(data)         |
                        | +Flush() (optional)  |
                        +----------------------+
                                  ^
                                  |
                      +-----------+------------+
                      |                        |
              +---------------+        +------------------+
              |  IRawStorage  |        |      IFile       |
              |---------------|        |-----------------|
              | +Write()      |        | +Write()        |
              | +Read()       |        | +Read()         |
              | +Flush()      |        | +Seek(pos)      |
              |               |        | +Tell()         |
              +---------------+        | +Close()        |
                                       | +Flush()        |
                                       +-----------------+
                                                ^
                                                |
                                       +------------------+
                                       |  IFileSystem      |
                                       |------------------|
                                       | +OpenFile(path)  |
                                       | +RemoveFile(path)|
                                       | +Exists(path)    |
                                       | +ListFiles()     |
                                       +------------------+

---

## Storage Manager (Optional)

- Central entry point to manage multiple backends
- Returns interfaces rather than implementing storage itself

struct Storage {
    static Storage& Instance() {
        static Storage s;
        return s;
    }

    IWriter* GetWriter(StorageType type) {
        switch(type) {
            case StorageType::FRAM: return &fram;
            case StorageType::EEPROM: return &eeprom;
            case StorageType::SD: return &sdFile; // optional file object
        }
        return nullptr;
    }

    IFile* OpenFile(const char* path) {
        return sdFs.OpenFile(path);
    }

private:
    FRAMStorage fram;
    EEPROMStorage eeprom;
    SDFileSystem sdFs;
    SDFile sdFile;
};

### Usage Examples

**Generic logging (don't care about backend):**

IWriter* writer = Storage::Instance().GetWriter(StorageType::FRAM);
writer->Write(data);
writer->Flush();

**Full file operations (SD only):**

IFile* file = Storage::Instance().OpenFile("log.txt");
file->Seek(0);
file->Write(data);
file->Close();

---

## Considerations

- `Flush()` is conceptually a file or buffered storage operation; for unbuffered raw storage it can be a no-op.
- `Storage` class is optional: 
  - Small projects → three globals (FRAM, EEPROM, SD) are fine.
  - Medium/large projects → `Storage` manager simplifies backend selection and initialization.
- Keep raw storage separate from filesystem storage; only filesystem supports `IFile` and `IFileSystem`.
- Generic code can operate on `IWriter` without knowing the concrete storage.
- Avoid mixing responsibilities in `Storage` (don't implement writing there, just return interfaces).
