#ifndef _TNFSLIB_MOUNTINFO_H
#define _TNFSLIB_MOUNTINFO_H

#include <cstdint>
#include <lwip/netdb.h>

#define TNFS_DEFAULT_PORT 16384
#define TNFS_RETRIES 5 // Number of times to retry if we fail to send/receive a packet
#define TNFS_TIMEOUT 6000 // We will not retry if this much time passes before reaching TNFS_RETRIES
#define TNFS_RETRY_DELAY 1000 // Default delay before retrying. Server will provide a minimum during TNFS_CMD_MOUNT
#define TNFS_MAX_FILE_HANDLES 8 // Max number of file handles we'll open to the server
#define TNFS_MAX_FILELEN 256

#define TNFS_INVALID_HANDLE -1
#define TNFS_INVALID_SESSION 0 // We're assuming a '0' is never a valid session ID

// Some things we need to keep track of for every file we open
struct tnfsFileHandleInfo
{
    uint8_t handle_id = 0;
    uint32_t position = 0;
    char filename[TNFS_MAX_FILELEN];
};

// Everything we need to know about and keep track of for the server we're talking to
class tnfsMountInfo
{
private:
    tnfsFileHandleInfo * _file_handles[TNFS_MAX_FILE_HANDLES] = { nullptr }; // Stored from server's responses to TNFS_OPEN

public:
    ~tnfsMountInfo();

    tnfsFileHandleInfo * new_filehandleinfo();
    tnfsFileHandleInfo * get_filehandleinfo(uint8_t filehandle);
    void delete_filehandleinfo(uint8_t filehandle);


    // These char[] sizes are abitrary...
    char hostname[64];
    in_addr_t host_ip = IPADDR_NONE;
    uint16_t port = TNFS_DEFAULT_PORT;
    char mountpath[64];
    char user[36];
    char password[36];
    uint16_t session = TNFS_INVALID_SESSION; // Stored from server's response to TNFS_MOUNT
    uint16_t min_retry_ms = TNFS_RETRY_DELAY; // Updated from server's response to TNFS_MOUNT
    uint16_t server_version = 0;  // Stored from server's response to TNFS_MOUNT
    uint8_t max_retries = TNFS_RETRIES;
    int timeout_ms = TNFS_TIMEOUT;
    uint8_t current_sequence_num = 0; // Updated with each transaction to the server
    int16_t dir_handle = TNFS_INVALID_HANDLE; // Stored from server's response to TNFS_OPENDIR
};

#endif // _TNFSLIB_MOUNTINFO_H