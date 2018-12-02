/*
 * Copyright (C) 2010-2014 NXP Semiconductors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * DAL I2C port implementation for linux
 *
 * Project: Trusted NFC Linux
 *
 */
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <errno.h>

#include <linux/gpio.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <phNxpLog.h>
#include <phTmlNfc_i2c.h>
#include <phNfcStatus.h>
#include <string.h>
#include "phNxpNciHal_utils.h"

#define CRC_LEN                     2
#define NORMAL_MODE_HEADER_LEN      3
#define FW_DNLD_HEADER_LEN          2
#define FW_DNLD_LEN_OFFSET          1
#define NORMAL_MODE_LEN_OFFSET      2
#define FRAGMENTSIZE_MAX            PHNFC_I2C_FRAGMENT_SIZE
static bool_t bFwDnldFlag = FALSE;

// ----------------------------------------------------------------------------
// Alternative use
// ----------------------------------------------------------------------------

#ifdef PHFL_TML_ALT_NFC
#include "phTmlNfc_alt.h"

// ----------------------------------------------------------------------------
// Global variables
// ----------------------------------------------------------------------------
static int iEnableFd    = 0;
static int iInterruptFd = 0;
static int iI2CFd       = 0;
static int gFd          = 0;
static int dummyHandle = 1234;

// ------------------------------------------------------------------
// Verify pin
// ------------------------------------------------------------------

#define EDGE_NONE    0
#define EDGE_RISING  1
#define EDGE_FALLING 2
#define EDGE_BOTH    3

#if GPIOCHARDEV

static int writegpiodev ( int fd, unsigned int line, unsigned int value ) {
    struct gpiohandle_request req;
    struct gpiohandle_data data;

    if ( fd <= 0 ) {
        // GPIO device not found
        NXPLOG_TML_E( "CHARDEV: GPIO device %s\n not available", GPIOCHARDEVNAME);
    } else {
        req.lines = 1;
        req.lineoffsets[0] = line;
        req.flags = GPIOHANDLE_REQUEST_OUTPUT;
	req.default_values[0] = value;
        strcpy(req.consumer_label, "PIN");
        int ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);
        if ( ret <= 0 ) {
            NXPLOG_TML_E( "CHARDEV: Could not issue GPIO_GET_LINEHANDLE_IOCTL to fd %d : %s", fd, strerror(errno) );
        }
        data.values[0] = value;
        ret = ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        if ( ret <= 0 ) {
            NXPLOG_TML_E( "CHARDEV: Could not write date to line %d use GPIOHANDLE_SET_LINE_VALUES_IOCTL:%s", line, strerror(errno) );
        } else {
            return( req.fd ); //return the line handler
        }
    }
    
}

static int readgpiodev ( int fd, unsigned int line ) {
    struct gpiohandle_request req;
    struct gpiohandle_data data;
    req.lineoffsets[0] = line;
    req.lines = 1;
    req.flags = GPIOHANDLE_REQUEST_INPUT;
    int lhfd = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);
    if ( req.fd <= 0 ) {
        NXPLOG_TML_E( "Could not trigger GPIO_GET_LINEHANDLE_IOCTL on fd'%d' (%s)", fd, strerror(errno) );
    }
    NXPLOG_TML_D( "readgpiodev:GPIO_GET_LINEHANDLE_IOCTL line %d",line );
    int ret = ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
    NXPLOG_TML_D( "readgpiodev:GPIOHANDLE_GET_LINE_VALUES_IOCTL, line %d",line );
    if ( ret < 0 ) {
        NXPLOG_TML_E( "Could not open value line '%d' (%s)", line, strerror(errno) );
        return -1;
    } else {
        NXPLOG_TML_D( "Pin %d value %s\n", line, data.values[0]?"high":"low" );
        return ret;
    }
}

void wait4interruptchardev(unsigned int line, uint32_t eventflag) {
    struct gpiohandle_request req;
    struct gpiohandle_data data;

    if (iInterruptFd < 0) {
        NXPLOG_TML_E( "GPIO device not available" );
    } else {

        req.lineoffsets[0] = line;
	req.lines = 1;
        req.flags = GPIOHANDLE_REQUEST_INPUT | eventflag;

        int ret = ioctl(iInterruptFd, GPIO_GET_LINEEVENT_IOCTL, &req);
        if (ret == -1) {
            NXPLOG_TML_E( "Failed to issue GET EVENT: IOCTL (%d)\n", ret );
	}
    }

    /* Read initial states */
    int ret = ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
    if (ret == -1) {
        NXPLOG_TML_E( "Failed to issue GPIOHANDLE GET LINE, VALUES IOCTL (%d)\n", ret);
    }

    NXPLOG_TML_D( "Monitoring line %d on %s\n", line, GPIOCHARDEVNAME );
    NXPLOG_TML_D( "Initial line value: %d\n", data.values[0] );

    while(1) {
        struct gpioevent_data event;

        ret = read(req.fd, &event, sizeof(event));
        if (ret == -1) {
            if (errno == -EAGAIN) {
                continue;
            } else {
                ret = -errno;
                NXPLOG_TML_E( "Failed to read event (%d)\n", ret );
                break;
            }
        }

        if (ret != sizeof(event)) {
            NXPLOG_TML_E( "Reading event failed\n");
            break;
        }
        NXPLOG_TML_D( "GPIO EVENT %llu: ", event.timestamp );
        if (event.id == eventflag) {
            NXPLOG_TML_D( "rising edge" );
            break;
        } else {
            NXPLOG_TML_D( "Not a rising edge, continue" );
            continue;
        }
    }
}


#endif
static int verifyPin( int pin, int isoutput, int edge ) {
    char buf[40];
    // Check if gpio pin has already been created
    int hasGpio = 0;
#if GPIOCHARDEV
    struct gpiochip_info cinfo;
    sprintf( buf, GPIOCHARDEVNAME );
    NXPLOG_TML_D( "GPIO device %s\n", GPIOCHARDEVNAME );
#else
    sprintf( buf, "/sys/class/gpio/gpio%d", pin );
    NXPLOG_TML_D( "Pin %s\n", buf );
#endif
    int fd = open( buf, O_RDONLY );
#if GPIOCHARDEV
    if ( fd < 0 ) {
        // GPIO device not found
        NXPLOG_TML_E( "CHARDEV: GPIO device %s\n not available: %s", buf, strerror(errno));
#else
    if ( fd <= 0 ) {
        // Pin not exported yet
        NXPLOG_TML_D( "Create pin %s\n", buf );
        if ( ( fd = open( "/sys/class/gpio/export", O_WRONLY ) ) > 0 ) {
            sprintf( buf, "%d", pin );
            if ( write( fd, buf, strlen(buf) ) == strlen(buf) ) {
                hasGpio = 1;
		usleep(100*1000);
            }
        }
#endif
    } else {
        NXPLOG_TML_E( "System already has pin %s\n", buf );
        hasGpio = 1;
    }
    close( fd );

    if ( hasGpio ) {
#if GPIOCHARDEV
        sprintf( buf, GPIOCHARDEVNAME );
        gFd = open( buf, O_RDONLY );
	if (gFd <= 0){
            NXPLOG_TML_E( "Failed to open '%s' (%s)", buf, strerror(errno) );
	} else {
	    NXPLOG_TML_D( "gFd %d", gFd );
	}
        int lfd;
        if ( isoutput ) {
	    NXPLOG_TML_D( "GPIOCHARDEV hasGpio isoutput" );
            lfd = writegpiodev( gFd, pin, 0 );
	    NXPLOG_TML_D( "GPIOCHARDEV hasGpio writegpiodev" );
            if ( lfd > 0 ) {
                NXPLOG_TML_D( "Pin %d now off\n", pin );
                return lfd;
            }
        }
	NXPLOG_TML_D( "GPIOCHARDEV not isoutput" );
        lfd = readgpiodev( gFd, pin );
        return gFd;
#else
        // Make sure it is an output
        sprintf( buf, "/sys/class/gpio/gpio%d/direction", pin );
        NXPLOG_TML_D( "Direction %s\n", buf );
        fd = open( buf, O_WRONLY );
        if ( fd <= 0 ) {
            NXPLOG_TML_E( "Could not open direction port '%s' (%s)", buf, strerror(errno) );
        } else {
            if ( isoutput ) {
                if ( write(fd,"out",3) == 3 ) {
                    NXPLOG_TML_D( "Pin %d now an output\n", pin );
                }
                close(fd);

                // Open pin and make sure it is off
                sprintf( buf, "/sys/class/gpio/gpio%d/value", pin );
                fd = open( buf, O_RDWR );
                if ( fd <= 0 ) {
                    NXPLOG_TML_E( "Could not open value port '%s' (%s)", buf, strerror(errno) );
                } else {
                    if ( write( fd, "0", 1 ) == 1 ) {
                        NXPLOG_TML_D( "Pin %d now off\n", pin );
                    }
                    return( fd );  // Success
                }
            } else {
                if ( write(fd,"in",2) == 2 ) {
                    NXPLOG_TML_D( "Pin %d now an input\n", pin );
                }
                close(fd);

                if ( edge != EDGE_NONE ) {
                    // Open pin edge control
                    sprintf( buf, "/sys/class/gpio/gpio%d/edge", pin );
                    NXPLOG_TML_D( "Edge %s\n", buf );
                    fd = open( buf, O_RDWR );
                    if ( fd <= 0 ) {
                        NXPLOG_TML_E( "Could not open edge port '%s' (%s)",    buf, strerror(errno) );
                    } else {
                        char * edge_str = "none";
                        switch ( edge ) {
                          case EDGE_RISING:  edge_str = "rising"; break;
                          case EDGE_FALLING: edge_str = "falling"; break;
                          case EDGE_BOTH:    edge_str = "both"; break;
                        }
                        int l = strlen(edge_str);
                        NXPLOG_TML_D( "Edge-string %s - %d\n", edge_str, l );
                        if ( write( fd, edge_str, l ) == l ) {
                            NXPLOG_TML_D( "Pin %d trigger on %s\n", pin, edge_str );
                        }
                        close(fd);
                    }    
                }
            
                // Open pin
                sprintf( buf, "/sys/class/gpio/gpio%d/value", pin );
                NXPLOG_TML_D( "Value %s\n", buf );
                fd = open( buf, O_RDONLY );
                if ( fd <= 0 ) {
                    NXPLOG_TML_E( "Could not open value port '%s' (%s)", buf, strerror(errno) );
                } else {
                    return( fd );  // Success
                }
            }
        }
#endif
    }
    return( 0 );
}

static void pnOn( void ) {
#if GPIOCHARDEV
    if ( iEnableFd ) writegpiodev( iEnableFd, PIN_ENABLE, 1 );
#else
    if ( iEnableFd ) write( iEnableFd, "1", 1 );
#endif
}

static void pnOff( void ) {
#if GPIOCHARDEV
    if ( iEnableFd ) writegpiodev( iEnableFd, PIN_ENABLE, 0 );
#else
    if ( iEnableFd ) write( iEnableFd, "0", 1 );
#endif

}

static int pnGetint( void ) {
    int len;
    char buf[2];
    
    if (iInterruptFd <= 0) {
        NXPLOG_TML_E( "Error with interrupt-detect pin (%d)", iInterruptFd );
        return( -1 );
    }

    // Seek to the start of the file
    lseek(iInterruptFd, SEEK_SET, 0);

    // Read the field_detect line
    len = read(iInterruptFd, buf, 2);

    if (len != 2) {
        NXPLOG_TML_E( "Error with interrupt-detect pin (%s)", strerror(errno));
        return( 0 );
    }

    return (buf[0] != '0');
}

static void wait4interrupt( void ) {
    /* Open STREAMS device. */
    struct pollfd fds[1];
    fds[0].fd = iInterruptFd;
    fds[0].events = POLLPRI;
    int timeout_msecs = -1;   // 100000;
    int ret;
    
    while (!pnGetint()) {
        // Wait for an edge on the GPIO pin to get woken up
        ret = poll(fds, 1, timeout_msecs);
        if ( ret != 1 ) {
          NXPLOG_TML_D( "wait4interrupt() %d - %s, ", ret, strerror(errno) );
        }
    }
}
#endif

/*******************************************************************************
**
** Function         phTmlNfc_i2c_close
**
** Description      Closes PN54X device
**
** Parameters       pDevHandle - device handle
**
** Returns          None
**
*******************************************************************************/
void phTmlNfc_i2c_close(void *pDevHandle)
{
#ifdef PHFL_TML_ALT_NFC
    if ( iEnableFd    ) close(iEnableFd);
    if ( iInterruptFd ) close(iInterruptFd);
    if ( iI2CFd       ) close(iI2CFd);
#else
    if (NULL != pDevHandle) close((intptr_t)pDevHandle);
#endif

    return;
}

/*******************************************************************************
**
** Function         phTmlNfc_i2c_open_and_configure
**
** Description      Open and configure PN54X device
**
** Parameters       pConfig     - hardware information
**                  pLinkHandle - device handle
**
** Returns          NFC status:
**                  NFCSTATUS_SUCCESS            - open_and_configure operation success
**                  NFCSTATUS_INVALID_DEVICE     - device open operation failure
**
*******************************************************************************/
NFCSTATUS phTmlNfc_i2c_open_and_configure(pphTmlNfc_Config_t pConfig, void ** pLinkHandle)
{
#ifdef PHFL_TML_ALT_NFC
    NXPLOG_TML_D("phTmlNfc_i2c_open_and_configure Alternative NFC\n");
    NXPLOG_TML_D( "NFC - Assign IO pins\n");
    // Assign IO pins
    NXPLOG_TML_D( "verifyPin PIN_INT\n\n");
    iInterruptFd = verifyPin( PIN_INT,    0, EDGE_RISING );
    NXPLOG_TML_D( "verifyPin PIN_ENABLE\n\n");
    iEnableFd    = verifyPin( PIN_ENABLE, 1, EDGE_NONE   );
    NXPLOG_TML_D( "NFCHW - open I2C bus - %s\n", I2C_BUS);

    // I2C bus
    iI2CFd = open( I2C_BUS, O_RDWR | O_NOCTTY);
    if (iI2CFd < 0) {
        NXPLOG_TML_E( "Could not open I2C bus '%s' (%s)", I2C_BUS, strerror(errno));
        if ( iEnableFd    ) close(iEnableFd);
        if ( iInterruptFd ) close(iInterruptFd);
        return( NFCSTATUS_INVALID_DEVICE );
    }

    NXPLOG_TML_D( "NFC - open I2C device - 0x%02x\n", I2C_ADDRESS);

    // I2C slave address
    if (ioctl(iI2CFd, I2C_SLAVE, I2C_ADDRESS) < 0) {
        NXPLOG_TML_E( "Cannot select I2C address (%s)\n", strerror(errno));
        if ( iEnableFd    ) close(iEnableFd);
        if ( iInterruptFd ) close(iInterruptFd);
        close(iI2CFd);
        return( NFCSTATUS_INVALID_DEVICE );
    }

    /*Reset NFC Controller */
    pnOn();
    usleep(100 * 1000);
    pnOff();
    usleep(100 * 1000);
    pnOn();
    
    *pLinkHandle = (void*) ((intptr_t)dummyHandle);
#else
    int nHandle;
    NXPLOG_TML_D("phTmlNfc_i2c_open_and_configure\n");
    NXPLOG_TML_D("Opening port=%s\n", pConfig->pDevName);
    /* open port */
    nHandle = open((char const *)pConfig->pDevName, O_RDWR);
    if (nHandle < 0)
    {
        NXPLOG_TML_E("_i2c_open() Failed: retval %x",nHandle);
        *pLinkHandle = NULL;
        return NFCSTATUS_INVALID_DEVICE;
    }

    *pLinkHandle = (void*) ((intptr_t)nHandle);

    /*Reset PN54X*/
    phTmlNfc_i2c_reset((void *)((intptr_t)nHandle), 1);
    usleep(100 * 1000);
    phTmlNfc_i2c_reset((void *)((intptr_t)nHandle), 0);
    usleep(100 * 1000);
    phTmlNfc_i2c_reset((void *)((intptr_t)nHandle), 1);
#endif

    return NFCSTATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         phTmlNfc_i2c_read
**
** Description      Reads requested number of bytes from PN54X device into given buffer
**
** Parameters       pDevHandle       - valid device handle
**                  pBuffer          - buffer for read data
**                  nNbBytesToRead   - number of bytes requested to be read
**
** Returns          numRead   - number of successfully read bytes
**                  -1        - read operation failure
**
*******************************************************************************/
int phTmlNfc_i2c_read(void *pDevHandle, uint8_t * pBuffer, int nNbBytesToRead)
{
    int ret_Read;
    int numRead = 0;
    uint16_t totalBtyesToRead = 0;
    
#ifdef PHFL_TML_ALT_NFC
  // Overwrite handle
  pDevHandle = (void*)iI2CFd;
#endif
  
    int ret_Select;
    struct timeval tv;
    fd_set rfds;

    UNUSED(nNbBytesToRead);
    if (NULL == pDevHandle)
    {
        return -1;
    }

    if (FALSE == bFwDnldFlag)
    {
        totalBtyesToRead = NORMAL_MODE_HEADER_LEN;
    }
    else
    {
        totalBtyesToRead = FW_DNLD_HEADER_LEN;
    }

    /* Read with 2 second timeout, so that the read thread can be aborted
       when the PN54X does not respond and we need to switch to FW download
       mode. This should be done via a control socket instead. */
    FD_ZERO(&rfds);
    FD_SET((intptr_t) pDevHandle, &rfds);
    tv.tv_sec = 2;
    tv.tv_usec = 1;

    ret_Select = select((int)((intptr_t)pDevHandle + (int)1), &rfds, NULL, NULL, &tv);
    if (ret_Select < 0)
    {
        NXPLOG_TML_E("i2c select() errno : %x",errno);
        return -1;
    }
    else if (ret_Select == 0)
    {
        NXPLOG_TML_E("i2c select() Timeout");
        return -1;
    }
    else
    {
#ifdef PHFL_TML_ALT_NFC
    #if GPIOCHARDEV
        wait4interruptchardev(PIN_INT, GPIOEVENT_EVENT_RISING_EDGE);
    #else
        wait4interrupt();
    #endif
#endif
        ret_Read = read((intptr_t)pDevHandle, pBuffer, totalBtyesToRead - numRead);
        if (ret_Read > 0)
        {
            numRead += ret_Read;
        }
        else if (ret_Read == 0)
        {
            NXPLOG_TML_E("_i2c_read() [hdr]EOF");
            return -1;
        }
        else
        {
            NXPLOG_TML_E("_i2c_read() [hdr] errno : %x",errno);
            return -1;
        }

        if (FALSE == bFwDnldFlag)
        {
            totalBtyesToRead = NORMAL_MODE_HEADER_LEN;
        }
        else
        {
            totalBtyesToRead = FW_DNLD_HEADER_LEN;
        }

        if(numRead < totalBtyesToRead)
        {
#ifdef PHFL_TML_ALT_NFC
    #if GPIOCHARDEV
        wait4interruptchardev(PIN_INT, GPIOEVENT_EVENT_RISING_EDGE);
    #else
        wait4interrupt();
    #endif
#endif
            ret_Read = read((intptr_t)pDevHandle, pBuffer, totalBtyesToRead - numRead);
            if (ret_Read != totalBtyesToRead - numRead)
            {
                NXPLOG_TML_E("_i2c_read() [hdr] errno : %x",errno);
                return -1;
            }
            else
            {
                numRead += ret_Read;
            }
        }
        if(TRUE == bFwDnldFlag)
        {
            totalBtyesToRead = pBuffer[FW_DNLD_LEN_OFFSET] + FW_DNLD_HEADER_LEN + CRC_LEN;
        }
        else
        {
            totalBtyesToRead = pBuffer[NORMAL_MODE_LEN_OFFSET] + NORMAL_MODE_HEADER_LEN;
        }
#ifdef PHFL_TML_ALT_NFC
    #if GPIOCHARDEV
        wait4interruptchardev(PIN_INT, GPIOEVENT_EVENT_RISING_EDGE);
    #else
        wait4interrupt();
    #endif
#endif
        ret_Read = read((intptr_t)pDevHandle, (pBuffer + numRead), totalBtyesToRead - numRead);
        if (ret_Read > 0)
        {
            numRead += ret_Read;
        }
        else if (ret_Read == 0)
        {
            NXPLOG_TML_E("_i2c_read() [pyld] EOF");
            return -1;
        }
        else
        {
            if(FALSE == bFwDnldFlag)
            {
                NXPLOG_TML_E("_i2c_read() [hdr] received");
                phNxpNciHal_print_packet("RECV",pBuffer, NORMAL_MODE_HEADER_LEN);
            }
            NXPLOG_TML_E("_i2c_read() [pyld] errno : %x",errno);
            return -1;
        }
    }
    return numRead;
}

/*******************************************************************************
**
** Function         phTmlNfc_i2c_write
**
** Description      Writes requested number of bytes from given buffer into PN54X device
**
** Parameters       pDevHandle       - valid device handle
**                  pBuffer          - buffer for read data
**                  nNbBytesToWrite  - number of bytes requested to be written
**
** Returns          numWrote   - number of successfully written bytes
**                  -1         - write operation failure
**
*******************************************************************************/
int phTmlNfc_i2c_write(void *pDevHandle, uint8_t * pBuffer, int nNbBytesToWrite)
{
#ifdef PHFL_TML_ALT_NFC
    // Overwrite handle
    pDevHandle = (void*)iI2CFd;
#endif
  
    int ret;
    int numWrote = 0;
    int numBytes = nNbBytesToWrite;
    if (NULL == pDevHandle)
    {
        return -1;
    }
    if(fragmentation_enabled == I2C_FRAGMENATATION_DISABLED && nNbBytesToWrite > FRAGMENTSIZE_MAX)
    {
        NXPLOG_TML_E("i2c_write() data larger than maximum I2C  size,enable I2C fragmentation");
        return -1;
    }
    while (numWrote < nNbBytesToWrite)
    {
        if(fragmentation_enabled == I2C_FRAGMENTATION_ENABLED && nNbBytesToWrite > FRAGMENTSIZE_MAX)
        {
            if(nNbBytesToWrite - numWrote > FRAGMENTSIZE_MAX)
            {
                numBytes = numWrote+ FRAGMENTSIZE_MAX;
            }
            else
            {
                numBytes = nNbBytesToWrite;
            }
        }
        ret = write((intptr_t)pDevHandle, pBuffer + numWrote, numBytes - numWrote);
        if (ret > 0)
        {
            numWrote += ret;
            if(fragmentation_enabled == I2C_FRAGMENTATION_ENABLED && numWrote < nNbBytesToWrite)
            {
                usleep(500);
            }
        }
        else if (ret == 0)
        {
            NXPLOG_TML_E("_i2c_write() EOF");
            return -1;
        }
        else
        {
            NXPLOG_TML_E("_i2c_write() errno : %x",errno);
            if (errno == EINTR || errno == EAGAIN)
            {
                continue;
            }
            return -1;
        }
    }

    return numWrote;
}

/*******************************************************************************
**
** Function         phTmlNfc_i2c_reset
**
** Description      Reset PN54X device, using VEN pin
**
** Parameters       pDevHandle     - valid device handle
**                  level          - reset level
**
** Returns           0   - reset operation success
**                  -1   - reset operation failure
**
*******************************************************************************/
#define PN54X_SET_PWR _IOW(0xe9, 0x01, unsigned int)
int phTmlNfc_i2c_reset(void *pDevHandle, long level)
{
#ifdef PHFL_TML_ALT_NFC
    switch ( level ) {
      case 2: 
          pnOn();
          usleep(100 * 1000);
          pnOff();
          usleep(100 * 1000);
          pnOn();
          break;
      case 1:  pnOn(); break;
      default: pnOff(); break;
    }
    return 0;
#else
    int ret;
    
    NXPLOG_TML_D("phTmlNfc_i2c_reset(), VEN level %ld", level);

    if (NULL == pDevHandle)
    {
        return -1;
    }

    ret = ioctl((intptr_t)pDevHandle, PN54X_SET_PWR, level);
    if(level == 2 && ret == 0)
    {
        bFwDnldFlag = TRUE;
    }else{
        bFwDnldFlag = FALSE;
    }
    return ret;
#endif
}

/*******************************************************************************
**
** Function         getDownloadFlag
**
** Description      Returns the current mode
**
** Parameters       none
**
** Returns           Current mode download/NCI
*******************************************************************************/
bool_t getDownloadFlag(void)
{
    return bFwDnldFlag;
}

