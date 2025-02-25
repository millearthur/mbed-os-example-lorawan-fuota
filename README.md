# Firmware-updates enabled LoRaWAN example application

This application implements multicast firmware updates over LoRaWAN for Mbed OS 5. It implements:

* [LoRaWAN Remote Multicast Setup Specification v1.0.0](https://lora-alliance.org/resource-hub/lorawan-remote-multicast-setup-specification-v100).
* [LoRaWAN Fragmented Data Block Transport Specification v1.0.0](https://lora-alliance.org/resource-hub/lorawan-fragmented-data-block-transport-specification-v100).
* [LoRaWAN Application Layer Clock Synchronization Specification v1.0.0](https://lora-alliance.org/resource-hub/lorawan-application-layer-clock-synchronization-specification-v100).
* [Status & Version Specification v0.2](https://github.com/millearthur/mbed-lorawan-update-client/LoRaWAN%20Version%20and%20Status%20Specification_v0_2.pdf).

* Delta updates: 
    1. Using JanPATCH (JDIIF/JPTCH)
    2. Using DDELTA & Compression (See [lorawan-update-client](https://github.com/millearthur/mbed-lorawan-update-client))
* Cryptographic verification of new firmware.
* Bootloader for flashing new firmware images.

Want to learn more? [Here's a video about the process](https://www.youtube.com/watch?v=0NoshDOqmdM), and [here's a demo of this application](https://www.youtube.com/watch?v=SSbDT1jxxwg&feature=youtu.be).

## Target configuration

This application runs on any Mbed-enabled development board, but requires some configuration. See the [porting guide](docs/porting-guide.md) for more information.

The storage layer and firmware slots are already present for the [L-TEK FF1705](https://os.mbed.com/platforms/L-TEK-FF1705/) and [DISCO-L475VG](https://os.mbed.com/platforms/ST-Discovery-L475E-IOT01A/) (with a radio shield) development boards. We suggest these boards if you want to test this solution out.

If you've added a new target configuration, please send a pull request to this repo!

## Build this application

### Mbed Tools
1. Install [Mbed CLI](https://os.mbed.com/docs/v5.10/tools/installation-and-setup.html) and the [GNU ARM Embedded Toolchain 6](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm).
1. Import this repository via:

    ```
    $ mbed import https://github.com/armmbed/mbed-os-example-lorawan-fuota
    ```
To use this repo please clone from forked : 

    $ mbed import https://github.com/millearthur/mbed-os-example-lorawan-fuota


1. In `main.cpp` specify your AppEui and AppKey.
1. In `mbed_app.json` specify your frequency plan (and [FSB](https://github.com/ARMmbed/mbed-os/blob/master/features/lorawan/FSB_Usage.txt)).


### Signing Tool
Next, you need to generate a public/private key pair. The public key is held in your application, and the private key is used to sign updates.

1. Install [Node.js 8 or higher](https://nodejs.org).
1. Install the [lorawan-fota-signing-tool](https://github.com/janjongboom/lorawan-fota-signing-tool) via:

    ```
    $ npm install lorawan-fota-signing-tool -g
    ```

    **Note:** this also requires Python 2.7 and OpenSSL installed.

1. Create a public/private key pair:

    ```
    $ lorawan-fota-signing-tool create-keypair -d yourdomain.com -m your-device-model-string
    ```

**Note** That a new version of the tool is available to be able to patch for the new DDELTA patching methode. (See [lorawan-fota-signing-tool](https://github.com/millearthur/lorawan-fota-signing-tool))

### Build
With everything configured, you can build the application.

1. Copy `.mbedignore_no_rtos` to `.mbedignore` - this removes the RTOS, and is required on targets with less than 32K SRAM (incl. L-TEK FF1705) without a crypto engine. The ECDSA/SHA256 verification will run out of memory if this is not done (`-0xfffffff0` error code). See [Build configuration](#build-configuration) for more information.
1. Build this application via:

    ```
    $ mbed compile -m auto -t GCC_ARM --profile=./profiles/tiny.json
    ```

1. Drag `BUILD/YOUR_TARGET/GCC_ARM-TINY/mbed-os-example-lorawan-fuota.bin` onto your development board (mounts as flash storage device).
1. When flashing is complete, attach a serial monitor on baud rate 115,200.

## Sending a first firmware update

After the device joined the network, and with the public key in place, you can send a firmware update.

1. Sign the example binary:

    ```
    $ lorawan-fota-signing-tool sign-binary -b example-firmware/xdot-blinky.bin -o xdot-blinky-signed.bin --output-format bin --override-version
    ```

1. Give `xdot-blinky-signed.bin` to your LoRaWAN network (differs per network) for a firmware update.

    **Note:** A testing script for LoRaServer.io is included with this repository, [see below](#testing-using-loraserverio).

The device will join a multicast session, receive the update, repair any missing packets, and then cryptographically verify the firmware using the public key. When this succeeds hit the **RESET** button, and the bootloader will update the firmware. After this, blinky will run.

## Creating a delta update

To create a delta update, re-flash mbed-os-example-lorawan-fuota.bin to your development board.

Then:

1. Back up your current `_update.bin` file. This is the application without the bootloader:

    ```
    $ mkdir updates
    $ cp BUILD/YOUR_TARGET/GCC_ARM-TINY/mbed-os-example-lorawan-fuota_update.bin updates/v1_update.bin
    ```

1. Make a small change in your application.
1. Compile the application.
1. Copy the new `_update.bin` file into the `updates` folder:

    ```
    $ cp BUILD/YOUR_TARGET/GCC_ARM-TINY/mbed-os-example-lorawan-fuota_update.bin updates/v2_update.bin
    ```

1. Create and sign the diff:

    ```
    $ lorawan-fota-signing-tool sign-delta --old updates/v1_update.bin --new updates/v2_update.bin --output-format bin -o updates/v1_to_v2.bin
    ```

1. Give `v1_to_v2.bin` to your LoRaWAN network (differs per network) for a firmware update.

The update will come in, and after updating the new program should run.

## DDELTA patch 
**Note** To use the DDELTA update format, you will need to download the [Ddelta tools](https://github.com/julian-klode/ddelta) and the [LZCL compressing tools](https://github.com/ChrisLomont/CompressionTools).

The Delta tool should be used to generate de diff file : 

    $ ./ddelta_generate v1/update.bin  v1/update.bin  v1v2_ddelta_patch.bin

The CompressionTool should be used to compressed the file : 

    $ CompressionTester.exe -c Arithmetic -i v1v2_ddelta_patch.bin -o v1v2_ddelta_patch.bin.ari

And finally sign the external patch created using the signing tool : 

    $ lorawan-fota-signing-tool sign-external-delta --old v1/update.bin --new v2/update.bin  -i v1v2_ddelta_patch.bin.arii -o v1v2_ddelta_patch.bin_signed


## Testing using LoRaServer.io

**NOTE** LoraServer became Chirpstack.io and this interaction method cannot be used anymore. If you are using Chirpstack and want to try the New FUOTA feature, you could use [Python Fuota Application](https://github.com/millearthur/python_app_fuota)

1. Follow the steps to install and configure LoRaServer.io, as described in [fuota-server/README.md](fuota-server/README.md).
1. Create fragments from a signed binary, via:

    ```
    $ lorawan-fota-signing-tool sign-binary -b example/xdot-blinky.bin -o fuota-server/xdot-blinky-signed.txt --frag-size 40 --redundancy-packets 20 --output-format packets-plain --override-version
    ```

1. Run:

    ```
    $ node fuota-server/loraserver.js fuota-server/xdot-blinky-signed.txt
    ```

**Note:** If you're using SF7, use a frag size of 204 (maximum size) for faster testing.

### Interop testing

The LoRa Alliance FUOTA test scenarios is also supported, but for this you need to enable interop mode. This will disable firmware verification (as it's not a real valid firmware going over the line).

**Note:** This does not override the GenAppKey. If you have changed the GenAppKey you need to change it back to the interop GenAppKey in main.cpp.

1. In `mbed_app.json`, set `lorawan-update-client.interop-testing` to `true`.
1. Create fragments for the test file, via:

    ```
    $ lorawan-fota-signing-tool create-frag-packets -i fuota-server/test-file.bin --output-format plain --frag-size 40 --redundancy-packets 5 -o fuota-server/test-file-unsigned.txt
    ```

1. Run:

    ```
    $ node fuota-server/loraserver.js fuota-server/test-file-unsigned.txt
    ```

**FlashIAPBlockDevice**

The interop tests require a lot less flash (only a few KB in slot 0) than the full update client. You can use the upper part of the internal flash as a scratch space in interop mode. See the [FlashIAPBlockDevice](https://os.mbed.com/docs/v5.10/apis/flashiapblockdevice.html) section in the Mbed OS documentation.

## Using the simulator for testing

You can test most things also in the simulator, including the interop tests against loraserver.

1. Install the [Mbed Simulator](https://github.com/janjongboom/mbed-simulator) v1.8 or higher.
1. From the 'mbed-os-example-lorawan-fuota' folder, run:

    ```
    $ LORA_HOST=LORASERVER_IP mbed-simulator .

    # LoRaWAN information:
    #         Gateway ID:             fe:00:89:00:00:29:cd:01
    ```

1. Register the gateway with that ID in LoRaServer.
1. Refresh the page with the simulator and the device should join the network.
1. Start the interop tests as you'd normally do.

## Application configuration

You can set some additional settings in `mbed_app.json`:

* `lorawan-update-client.overwrite-version` - the manifest contains the build date of the binary, and binaries that are older than the current firmware are rejected. You might not want this in testing. Set to `true` to overwrite the version at runtime.
* `lorawan-update-client.interop-testing` - skips firmware verification and writing the bootloader header. In addition this will start broadcasting the CRC32 hash of the received file after receiving the full file. Use this for interop testing with the LoRa Alliance FUOTA test scenarios.

## Build configuration

For optimized builds you can build without the RTOS enabled, with newlib-nano, and a different printf library. Some background is in [this blog post](https://os.mbed.com/blog/entry/Reducing-memory-usage-with-a-custom-prin/). To do this:

1. Rename `.mbedignore_no_rtos` to `.mbedignore`.
1. Build with:

    ```
    $ mbed compile --profile=./profiles/tiny.json
    ```

On the L-TEK FF1705 this consumes:

```
Total Static RAM memory (data + bss): 10712 bytes
Total Flash memory (text + data): 109254 bytes
```

## Memory usage

Memory usage also depends on the presence of the RTOS.

Baseline numbers on the FF1705, which include Mbed OS + RTOS + the LoRaWAN stack + the Update Client:

```
Static RAM: 19560 bytes
Heap size: 5384 bytes
Free: 6480 bytes
```

Without RTOS, with newlib-nano, and with tiny profile:

```
Static RAM: 10600 bytes
Heap size: 4812 bytes
Free: 16488 bytes
```

In addition, memory is used during the firmware update. This is dynamically allocated memory to reconstruct the firmware, and do cryptographic verification of the image. Details on the memory usage, and memory pressure events that your application can subscribe to are found in [mbed-lorawan-update-client#memory-usage](https://github.com/janjongboom/mbed-lorawan-update-client#memory-usage).

## Automated testing

See [mbed-lorawan-update-client#unit-tests](https://github.com/janjongboom/mbed-lorawan-update-client#unit-tests) to configure your storage layer. No changes are required for the L-TEK FF1705.

If you want to run the tests from this folder:

1. Copy the UpdateCerts.h file from the test folder:

    ```
    $ cp mbed-lorawan-update-client/TESTS/COMMON/UpdateCerts.h .
    ```

1. Create the following `.mbedignore` file:

    ```
    source/main.cpp
    source/example_insecure_rot.c
    ```

1. And run the tests:

    ```
    $ mbed test --app-config mbed-lorawan-update-client/TESTS/tests/mbed_app.json -n mbed-lorawan-update-client-tests-tests-* -v
    ```

## Manifest format and update signing procedure

**Note:** We're planning to switch to the IETF SUIT manifest when it's ready, the current manifest format is a stop-gap solution.

Every firmware update is accompanied with a manifest, a file with metadata about the update. This manifest contains a cryptographic signature, information on which devices the update can be applied to, and whether it's a delta update. Updates are signed with the private key of an Elliptic Curve key pair using ECDSA/SHA256. This means that the SHA256 hash of the firmware is signed. This signature is then placed in the manifest. When doing a firmware update, the new binary (outcome after patching) is signed, thus also acting as a way to verify that patching was successful.

### Update format

The update file format is:

1. The update file (either diff or full image).
1. 1 byte, size of the signature (70, 71 or 72 bytes).
1. 72 bytes, ECDSA/SHA256 signature of the update file. In case of a patch file, this is the signature of the file *after* patching (thus it's also a way of checking if patching succeeded). If the signature is smaller than 72 bytes, right pad with `00`.
1. 16 bytes, manufacturer UUID.
1. 16 bytes, device class UUID.
1. 4 bytes, version - this is the last modified data of the update file.
1. 1 byte, diff indication. If `0`, then this is not a delta update. If `1` it's a delta update.
1. 3 bytes, size of current firmware (if delta update). If sending a delta update then this field indicates the size of the current (before patching) firmware.

### New Update Format

Using the new DDELTA patching methods, a new patch format is defined. The *`diff_info[0]`* byte can hold the value `2` that specifies that the update is using a new DDELTA format. The patch will then be handled by the correct algorithms. (See [lorawan-update-client](https://github.com/millearthur/mbed-lorawan-update-client))

## Building a small firmware image for testing

It's useful to have a small, valid firmware image during testing (similar to `example-firmware/xdot-blinky.bin`) to quickly test the full update flow. To build one:

1. Clone [mbed-os-example-blinky-no-rtos](https://github.com/janjongboom/mbed-os-example-blinky-no-rtos).
1. Open `mbed_app.json` and add a new section under `target_overrides`:

    ```json
    {
        "target_overrides": {
            "*": {
                "platform.stdio-flush-at-exit": false
            },
            "YOUR_TARGET_NAME": {
                "target.features_add": ["BOOTLOADER"],
                "target.app_offset": "0x8400",
                "target.header_offset": "0x8000",
                "target.bootloader_img": "bootloader/YOUR_TARGET_NAME.bin"
            }
        }
    }
    ```

    **Note:** `app_offset` and `header_offset` need to be the same as in 'mbed-os-example-lorawan-fuota'.

1. Build the application:

    ```
    $ mbed compile -m YOUR_TARGET_NAME -t GCC_ARM --profile=./profiles/tiny.json
    ```

    **Note:** Compiling with ARMCC typically yields smaller binaries.

1. Find `BUILD/YOUR_TARGET_NAME/GCC_ARM-TINY/mbed-os-example-blinky-no-rtos_update.bin`. This is a binary you can send over a firmware update.


## DevEui Helper

In order to help differenciate the devices when using the DISCO platform, a new `DEVEUI` helper file has been added. Since the radio used in this case does not contain a integrated DevEui, a stable Fake EUI will be generated using the Bluetooth address of the board. This requires to build the Bluetooth drivers for the device.

This makes the solution quite heavy to compile, but this feature can be discarded if the board used is not the same, or if the radio module supports an integrated DevEui.


## Mbed OS version

This application is built on Mbed OS 5.11, but requires a single patch for session serialization ([here](https://github.com/janjongboom/mbed-os/commit/271c33d63f6cb1c01de7dd983552ab4af435d9af)). However, we believe that this is a sub-optimal solution that probably has some race conditions and are working to integrate a better solution into Mbed OS core.
