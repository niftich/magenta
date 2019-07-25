/*
 * QR Code generator demo (C++)
 *
 * Run this command-line program with no arguments. The program computes a bunch of demonstration
 * QR Codes and prints them to the console. Also, the SVG code for one QR Code is printed as a sample.
 *
 * Copyright (c) 2016 Project Nayuki
 * https://www.nayuki.io/page/qr-code-generator-library
 *
 * (MIT License)
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * - The above copyright notice and this permission notice shall be included in
 *   all copies or substantial portions of the Software.
 * - The Software is provided "as is", without warranty of any kind, express or
 *   implied, including but not limited to the warranties of merchantability,
 *   fitness for a particular purpose and noninfringement. In no event shall the
 *   authors or copyright holders be liable for any claim, damages or other
 *   liability, whether in an action of contract, tort or otherwise, arising from,
 *   out of or in connection with the Software or the use or other dealings in the
 *   Software.
 */

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <qrcodegen/qrcode.h>


// Function prototypes
static void doBasicDemo();
static void doVarietyDemo();
static void doSegmentDemo();
static void printQr(const qrcodegen::QrCode &qr);



// The main application program.
int main(int argc, char **argv) {
    doBasicDemo();
    doVarietyDemo();
    doSegmentDemo();
    return 0;
}


// Creates a single QR Code, then prints it to the console.
static void doBasicDemo() {
    const char *text = "Hello, world!";  // User-supplied text
    const qrcodegen::Ecc errCorLvl = qrcodegen::Ecc::LOW;  // Error correction level

    // Make and print the QR Code symbol
    qrcodegen::QrCode qr;
    qr.encodeText(text, errCorLvl);
    printQr(qr);
}


// Creates a variety of QR Codes that exercise different features of the library, and prints each one to the console.
static void doVarietyDemo() {
    // Project Nayuki URL
    qrcodegen::QrCode qr0;
    qr0.encodeText("https://www.nayuki.io/", qrcodegen::Ecc::HIGH);
    qr0.changeMask(3);  // Change mask, forcing to mask #3
    printQr(qr0);

    // Numeric mode encoding (3.33 bits per digit)
    qrcodegen::QrCode qr1;
    qr1.encodeText("314159265358979323846264338327950288419716939937510", qrcodegen::Ecc::MEDIUM);
    printQr(qr1);

    // Alphanumeric mode encoding (5.5 bits per character)
    qrcodegen::QrCode qr2;
    qr2.encodeText("DOLLAR-AMOUNT:$39.87 PERCENTAGE:100.00% OPERATIONS:+-*/", qrcodegen::Ecc::HIGH);
    printQr(qr2);

    // Unicode text as UTF-8, and different masks
    qrcodegen::QrCode qr3;
    qr3.encodeText("\xE3\x81\x93\xE3\x82\x93\xE3\x81\xAB\xE3\x81\xA1wa\xE3\x80\x81\xE4\xB8\x96\xE7\x95\x8C\xEF\xBC\x81\x20\xCE\xB1\xCE\xB2\xCE\xB3\xCE\xB4", qrcodegen::Ecc::QUARTILE);
    qr3.changeMask(0);
    printQr(qr3);
    qr3.changeMask(1);
    printQr(qr3);
    qr3.changeMask(5);
    printQr(qr3);
    qr3.changeMask(7);
    printQr(qr3);

    // Moderately large QR Code using longer text (from Lewis Carroll's Alice in Wonderland)
    qrcodegen::QrCode qr4;
    qr4.encodeText(
        "Alice was beginning to get very tired of sitting by her sister on the bank, "
        "and of having nothing to do: once or twice she had peeped into the book her sister was reading, "
        "but it had no pictures or conversations in it, 'and what is the use of a book,' thought Alice "
        "'without pictures or conversations?' So she was considering in her own mind (as well as she could, "
        "for the hot day made her feel very sleepy and stupid), whether the pleasure of making a "
        "daisy-chain would be worth the trouble of getting up and picking the daisies, when suddenly "
        "a White Rabbit with pink eyes ran close by her.", qrcodegen::Ecc::HIGH);
    printQr(qr4);
}


// Creates QR Codes with manually specified segments for better compactness.
static void doSegmentDemo() {
    // Illustration "silver"
    const char *silver0 = "THE SQUARE ROOT OF 2 IS 1.";
    const char *silver1 = "41421356237309504880168872420969807856967187537694807317667973799";
    qrcodegen::QrCode qr0;
    qr0.encodeText(
        (std::string(silver0) + silver1).c_str(),
        qrcodegen::Ecc::LOW);
    printQr(qr0);

    std::vector<qrcodegen::QrSegment> segs;
    segs.push_back(qrcodegen::QrSegment::makeAlphanumeric(silver0));
    segs.push_back(qrcodegen::QrSegment::makeNumeric(silver1));
    qrcodegen::QrCode qr1;
    qr1.encodeSegments(segs, qrcodegen::Ecc::LOW);
    printQr(qr1);

    // Illustration "golden"
    const char *golden0 = "Golden ratio \xCF\x86 = 1.";
    const char *golden1 = "6180339887498948482045868343656381177203091798057628621354486227052604628189024497072072041893911374";
    const char *golden2 = "......";
    qrcodegen::QrCode qr2;
    qr2.encodeText(
        (std::string(golden0) + golden1 + golden2).c_str(),
        qrcodegen::Ecc::LOW);
    printQr(qr2);

    segs.clear();
    std::vector<uint8_t> bytes;
    for (const char *temp = golden0; *temp != '\0'; temp++)
        bytes.push_back(static_cast<uint8_t>(*temp));
    segs.push_back(qrcodegen::QrSegment::makeBytes(bytes));
    segs.push_back(qrcodegen::QrSegment::makeNumeric(golden1));
    segs.push_back(qrcodegen::QrSegment::makeAlphanumeric(golden2));
    qrcodegen::QrCode qr3;
    qr3.encodeSegments(segs, qrcodegen::Ecc::LOW);
    printQr(qr3);
}


// Prints the given QR Code to the console.
static void printQr(const qrcodegen::QrCode &qr) {
    int border = 4;
    for (int y = -border; y < qr.size() + border; y++) {
        for (int x = -border; x < qr.size() + border; x++) {
            std::cout << (qr.pixel(x, y) == 1 ? "##" : "  ");
        }
        std::cout << std::endl;
    }
}
