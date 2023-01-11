project(CFE_MPL3115A2 C)

# Create the app module
add_cfe_app(mpl3115a2 fsw/src/mpl3115a2.c)

# The API to this library (which may be invoked/referenced from other apps)
# is stored in fsw/public_inc.
target_include_directories(mpl3115a2 PUBLIC fsw/public_inc)
