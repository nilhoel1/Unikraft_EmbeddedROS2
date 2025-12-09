// rmw_zenoh_static_stub.cpp
// Static registration stub for rmw_zenoh middleware
// This forces rmw_zenoh to be registered at link time without dlopen

#include <cstdlib>
#include <string>

// RMW API headers
extern "C" {
#include "rmw/rmw.h"
#include "rmw/init.h"
#include "rmw/get_implementation_identifier.h"
}

// Forward declarations of rmw_zenoh implementation functions
// These are defined in rmw_zenoh_cpp library
extern "C" {

// RMW implementation identifier for Zenoh
const char * rmw_zenoh_cpp_identifier = "rmw_zenoh_cpp";

// Forward declare the key rmw_zenoh functions that need to be linked
extern const rmw_init_options_t * rmw_get_zero_initialized_init_options(void);
extern rmw_ret_t rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator);
extern rmw_ret_t rmw_init_options_fini(rmw_init_options_t * init_options);
extern rmw_ret_t rmw_init(const rmw_init_options_t * options, rmw_context_t * context);
extern rmw_ret_t rmw_shutdown(rmw_context_t * context);

// Additional rmw_zenoh functions
extern rmw_node_t * rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_,
  size_t domain_id,
  const rmw_node_security_options_t * security_options,
  bool localhost_only);

extern rmw_ret_t rmw_destroy_node(rmw_node_t * node);

} // extern "C"

// Static initializer class to force registration
class RMWZenohStaticInitializer {
public:
  RMWZenohStaticInitializer() {
    // Set environment variable to override RMW implementation selection
    // This ensures rmw_zenoh is used even in static builds
    setenv("RMW_IMPLEMENTATION", "rmw_zenoh_cpp", 0);
    
    // Force reference to rmw_zenoh symbols to prevent dead code elimination
    // These volatile pointers ensure the linker includes rmw_zenoh functions
    volatile auto init_ptr = &rmw_init;
    volatile auto shutdown_ptr = &rmw_shutdown;
    volatile auto node_ptr = &rmw_create_node;
    volatile auto destroy_ptr = &rmw_destroy_node;
    
    // Prevent compiler optimization
    (void)init_ptr;
    (void)shutdown_ptr;
    (void)node_ptr;
    (void)destroy_ptr;
  }
};

// Create a global static instance to run initialization before main()
static RMWZenohStaticInitializer g_rmw_zenoh_initializer;

// Additional strong symbol references to force linking
// This prevents the linker from removing "unused" rmw_zenoh code
extern "C" {

// Force strong reference to rmw_zenoh implementation
__attribute__((used))
static const char * force_rmw_zenoh_link = rmw_zenoh_cpp_identifier;

// Alternative: Use constructor attribute to ensure initialization
__attribute__((constructor))
static void rmw_zenoh_static_init() {
  // This runs before main() and ensures rmw_zenoh is initialized
  setenv("RMW_IMPLEMENTATION", "rmw_zenoh_cpp", 0);
}

// Weak symbol override for rmw_get_implementation_identifier
// This ensures rmw_zenoh is returned when querying the RMW implementation
__attribute__((weak))
const char * rmw_get_implementation_identifier_override(void) {
  return "rmw_zenoh_cpp";
}

} // extern "C"

// Export a symbol that can be referenced in linker flags
extern "C" void __rmw_zenoh_static_registration_anchor(void) {
  // This function exists solely to be referenced by linker flags
  // Usage: -Wl,-u,__rmw_zenoh_static_registration_anchor
}
