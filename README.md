# FRC 2024
![img.png](CrescendoLogo.png)

This is FRC 5160's robot code for the 2024 season.

Some features of our codebase include:

1. Custom units library with minimal runtime overhead([Kmeasure](https://github.com/battery-staple/KMeasure)).
2. Vendor hardware wrappers which allow for easy swapping of different hardware components(and which support kmeasure)
3. Advantagekit log and replay support; handled by a property-delegate driven wrapper that also supports kotlin nullables, kmeasure and other kotlin data types
4. Swerve with pathplanner support, motion profiling and more
5. A custom command DSL which combines the best parts of subclassed and inline commands.
