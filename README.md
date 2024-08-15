# Object Bounding Box Plugin for Blender

This Blender plugin allows you to find the approximate minimum bounding box of a mesh object. The bounding box is a box that tightly surrounds the object, which can be useful for various applications such as physics simulations, packaging, and more.

## Installation

1. Download the `object_bounding_box.py` file.
2. Open Blender.
3. Go to `Edit > Preferences`.
4. In the Preferences window, select the `Add-ons` tab on the left side.
5. Click on the `Install...` button at the top right.
6. Locate and select the `object_bounding_box.py` file you downloaded.
7. Check the box next to the "Object Bounding Box" plugin to enable it.

## Usage

Once the plugin is installed and enabled, follow these steps to use it:

1. **Select an Object**:
    - In the 3D Viewport, select the mesh object for which you want to find the minimum bounding box.

2. **Invoke the Operator**:
    - Press `F3` (or `Space` in some versions) to bring up the search menu.
    - Type "Min Bounding Box" and select the `Min Bounding Box` option from the list.
    
3. **Adjust the Settings**:
    - A dialog box will appear allowing you to adjust various parameters:
        - **Visualize Sample**: Adds a sphere to the scene showing random direction samples (default: `False`).
        - **Visualize Boxes**: Adds cubes for all bounding boxes tried (can make the scene messy, default: `False`).
        - **Area Samples**: The number of random directions to test calipers in (default: `200`).
        - **Angular Sample**: The angular step to rotate calipers. For example, `90` means 1-degree steps, `180` means 0.5-degree steps (default: `50`).
        - **Method**: The method used to find the bounding box:
            - `BRUTE FORCE`: Checks a bunch of random boxes (default).
            - `PCAY`: Good for linear objects.
            - `PCAX`: Good for flat objects.
            - `PCAZ`: Good for some other objects.
    - Adjust these settings as needed, then click `OK`.

4. **Result**:
    - A new cube object representing the minimum bounding box will be added to the scene.
    - The bounding box object will be aligned and scaled to match the minimum bounding box of the selected mesh object.

## Notes

- The plugin uses a brute-force approach or principal component analysis (PCA) to find the minimum bounding box, depending on the selected method.
- The generated bounding box will be added to the same collection as the selected object.

## License

This plugin is provided under the MIT License. See the `LICENSE` file for more information.

## Author

- **Patrick R. Moore**

If you encounter any issues or have questions, feel free to open an issue on the repository.

