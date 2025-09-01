#ifndef NETWORKPROTOCOLFTP_H
#define NETWORKPROTOCOLFTP_H

#include "FS.h"

#include "fnFTP.h"

class NetworkProtocolFTP : public NetworkProtocolFS
{
public:
    /**
     * @brief ctor
     * @param rx_buf pointer to receive buffer
     * @param tx_buf pointer to transmit buffer
     * @param sp_buf pointer to special buffer
     * @return a NetworkProtocolFS object
     */
    NetworkProtocolFTP(std::string *rx_buf, std::string *tx_buf, std::string *sp_buf);

    /**
     * dTOR
     */
    virtual ~NetworkProtocolFTP();

    /**
     *  Class 'NetworkProtocolFTP' does not have a copy constructor which is recommended since it has dynamic memory/resource allocation(s).
     * Unless these two functions are implemented, they are being deleted so they cannot be used
     */
    NetworkProtocolFTP (const NetworkProtocolFTP&) = delete;
    NetworkProtocolFTP& operator= (const NetworkProtocolFTP&) = delete;

    /**
     * @brief Return a DSTATS byte for a requested COMMAND byte.
     * @param cmd The Command (0x00-0xFF) for which DSTATS is requested.
     * @return a 0x00 = No payload, 0x40 = Payload to Atari, 0x80 = Payload to FujiNet, 0xFF = Command not supported.
     */
    virtual uint8_t special_inquiry(uint8_t cmd);

    /**
     * @brief execute a command that returns no payload
     * @param cmdFrame a pointer to the passed in command frame for aux1/aux2/etc
     * @return error flag. TRUE on error, FALSE on success.
     */
    virtual bool special_00(cmdFrame_t *cmdFrame);

    /**
     * @brief execute a command that returns a payload to the atari.
     * @param sp_buf a pointer to the special buffer
     * @param len Length of data to request from protocol. Should not be larger than buffer.
     * @return error flag. TRUE on error, FALSE on success.
     */
    virtual bool special_40(uint8_t *sp_buf, unsigned short len, cmdFrame_t *cmdFrame);

    /**
     * @brief execute a command that sends a payload to fujinet (most common, XIO)
     * @param sp_buf, a pointer to the special buffer, usually a EOL terminated devicespec.
     * @param len length of the special buffer, typically SPECIAL_BUFFER_SIZE
     */
    virtual bool special_80(uint8_t *sp_buf, unsigned short len, cmdFrame_t *cmdFrame);

    /**
     * @brief Rename file specified by incoming devicespec.
     * @param url pointer to PeoplesUrlParser pointing to file/dest to rename
     * @param cmdFrame the command frame
     * @return TRUE on error, FALSE on success
     */
    virtual bool rename(PeoplesUrlParser *url, cmdFrame_t *cmdFrame);

    /**
     * @brief Delete file specified by incoming devicespec.
     * @param url pointer to PeoplesUrlParser pointing to file to delete
     * @param cmdFrame the command frame
     * @return TRUE on error, FALSE on success
     */
    virtual bool del(PeoplesUrlParser *url, cmdFrame_t *cmdFrame);

    /**
     * @brief Make directory specified by incoming devicespec.
     * @param url pointer to PeoplesUrlParser pointing to file to delete
     * @param cmdFrame the command frame
     * @return TRUE on error, FALSE on success
     */
    virtual bool mkdir(PeoplesUrlParser *url, cmdFrame_t *cmdFrame);

    /**
     * @brief Remove directory specified by incoming devicespec.
     * @param url pointer to PeoplesUrlParser pointing to file to delete
     * @param cmdFrame the command frame
     * @return TRUE on error, FALSE on success
     */
    virtual bool rmdir(PeoplesUrlParser *url, cmdFrame_t *cmdFrame);

    /**
     * @brief lock file specified by incoming devicespec.
     * @param url pointer to PeoplesUrlParser pointing to file to delete
     * @param cmdFrame the command frame
     * @return TRUE on error, FALSE on success
     */
    virtual bool lock(PeoplesUrlParser *url, cmdFrame_t *cmdFrame);

    /**
     * @brief unlock file specified by incoming devicespec.
     * @param url pointer to PeoplesUrlParser pointing to file to delete
     * @param cmdFrame the command frame
     * @return TRUE on error, FALSE on success
     */
    virtual bool unlock(PeoplesUrlParser *url, cmdFrame_t *cmdFrame);

protected:
    /**
     * Is rename implemented?
     */
    bool rename_implemented = true;

    /**
     * Is delete implemented?
     */
    bool delete_implemented = true;

    /**
     * Is mkdir implemented?
     */
    bool mkdir_implemented = true;

    /**
     * Is rmdir implemented?
     */
    bool rmdir_implemented = true;

    /**
     * @brief Open file handle, set fd
     * @return FALSE if successful, TRUE on error.
     */
    virtual bool open_file_handle();

    /**
     * @brief Open directory handle
     * @return FALSE if successful, TRUE on error.
     */
    virtual bool open_dir_handle();

    /**
     * @brief Do FTP mount
     * @param url the URL to mount
     * @return false on no error, true on error.
     */
    virtual bool mount(PeoplesUrlParser *url);

    /**
     * @brief Unmount FTP server specified in mountInfo.
     * @return  false on no error, true on error.
     */
    virtual bool umount();

    /**
     * @brief Translate filesystem error codes to Atari error codes. Sets error in Protocol.
     */
    virtual void fserror_to_error();

    /**
     * @brief Read from file handle
     * @param buf target buffer
     * @param len the number of bytes requested
     * @return FALSE if success, TRUE if error
     */
    virtual bool read_file_handle(uint8_t *buf, unsigned short len);

    /**
     * @brief read next directory entry.
     * @param buf the target buffer
     * @param len length of target buffer
     */
    virtual bool read_dir_entry(char *buf, unsigned short len);

    /**
     * @brief for len requested, break up into number of required
     *        FTP_write() blocks.
     * @param len Requested # of bytes.
     * @return TRUE on error, FALSE on success.
     */
    virtual bool write_file_handle(uint8_t *buf, unsigned short len);

    /**
     * @brief close file handle
     * @return FALSE if successful, TRUE on error.
     */
    virtual bool close_file_handle();

    /**
     * @brief Close directory handle
     * @return FALSE if successful, TRUE on error.
     */
    virtual bool close_dir_handle();

    /**
     * @brief return status from file (e.g. # of bytes remaining.)
     * @param Pointer to NetworkStatus object to inject new data.
     * @return FALSE if success, TRUE if error.
     */
    virtual bool status_file(NetworkStatus *status);

private:
    /**
     * fnFTP instance
     */
    fnFTP *ftp;

    /**
     * TRUE = STOR, FALSE = RETR
     */
    bool stor = false;

    /**
     * @brief get status of file, filling in filesize. mount() must have already been called.
     */
    virtual bool stat();
};

#endif /* NETWORKPROTOCOLFTP_H */