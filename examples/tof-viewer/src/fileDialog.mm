#import <Foundation/Foundation.h>
#import <Cocoa/Cocoa.h>
#include <string>
#include <vector>

std::vector<std::string>openFileDialog(
    char const * const aTitle ,
    char const * const aDefaultPathAndFile ,
    const std::vector<std::string> & filters) {

    int i;
    std::vector<std::string> fileList;
    NSOpenPanel* openDlg = [NSOpenPanel openPanel];
    [openDlg setLevel:CGShieldingWindowLevel()];

    // Set array of file types
    NSMutableArray * fileTypesArray = [NSMutableArray array];
    for (i = 0;i < filters.size(); i++) {
        NSString * filt =[NSString stringWithUTF8String:filters[i].c_str()];
        [fileTypesArray addObject:filt];
    }

    // Enable options in the dialog.
    [openDlg setCanChooseFiles:YES];
    [openDlg setCanChooseDirectories:NO];
    [openDlg setAllowedFileTypes:fileTypesArray];
    [openDlg setAllowsMultipleSelection:FALSE];
    [openDlg setDirectoryURL:[NSURL URLWithString:[NSString stringWithUTF8String:aDefaultPathAndFile ] ] ];

    // Display the dialog box. If the OK pressed return 1 filname
    if ( [openDlg runModal] == NSModalResponseOK ) {
        NSArray *files = [openDlg URLs];
        for( i = 0; i < [files count]; i++ ) {
            fileList.push_back(std::string([[[files objectAtIndex:i] path] UTF8String]));
        }
    }
    return fileList;
}

std::vector<std::string>saveFileDialog(
        char const * const aTitle ,
        char const * const aDefaultPathAndFile ,
        const std::vector<std::string> & filters) {

    int i;
    std::vector<std::string> fileList;
    // Create a File Save Dialog class.
    NSSavePanel* saveDlg = [NSSavePanel savePanel];
    [saveDlg setLevel:CGShieldingWindowLevel()];

    NSMutableArray * fileTypesArray = [NSMutableArray array];
    for (i = 0;i < filters.size(); i++) {
        NSString * filt =[NSString stringWithUTF8String:filters[i].c_str()];
        [fileTypesArray addObject:filt];
    }

    // Enable options in the dialog.
    [saveDlg setAllowedFileTypes:fileTypesArray];
    [saveDlg setDirectoryURL:[NSURL URLWithString:[NSString stringWithUTF8String:aDefaultPathAndFile ] ] ];

    // Display the dialog box. Return filename if OK pressed
    if ( [saveDlg runModal] == NSModalResponseOK ) {
        NSURL *file = [saveDlg URL];
        if (file) {
        fileList.push_back(std::string([[file path] UTF8String]));
        }
    }
    return fileList;
}
