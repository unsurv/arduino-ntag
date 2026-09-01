#ifndef NdefMessage_h
#define NdefMessage_h

#include <Ndef.h>
#include <NdefRecord.h>

#define MAX_NDEF_RECORDS 4
#define KNOWN_TYPE_HTTPS 0x04
#define KNOWN_TYPE_HTTP  0x03

class NdefMessage
{
    public:
        NdefMessage(void);
        NdefMessage(const byte *data, const int numBytes);
        NdefMessage(const NdefMessage& rhs);
        ~NdefMessage();
        NdefMessage& operator=(const NdefMessage& rhs);

        int getEncodedSize(); // need so we can pass array to encode
        void encode(byte *data);

        boolean addRecord(NdefRecord& record);
        void addMimeMediaRecord(String mimeType, String payload);
        void addMimeMediaRecord(String mimeType, byte *payload, int payloadLength);
        void addTextRecord(String text);
        void addTextRecord(String text, String encoding);
        void addUriRecord(String uri);
        void addUrlRecord(byte knownTypeUrlPrefix, String url);
        void addUrlRecordTemp(byte knownTypeUrlPrefix, String url);
        void addUnknownRecord(byte *payload, int payloadLength);
        void addEmptyRecord();

        unsigned int getRecordCount();
        NdefRecord getRecord(int index);
        NdefRecord operator[](int index);

        void print();
    private:
        NdefRecord _records[MAX_NDEF_RECORDS];
        unsigned int _recordCount;
};

#endif
