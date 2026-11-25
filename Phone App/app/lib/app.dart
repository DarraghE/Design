import 'dart:async';
import 'dart:io';

import 'package:bike_navigation/bike_navigation.dart';
import 'package:material_ui/material_ui.dart';
import 'package:path_provider/path_provider.dart';

import 'closepass/close_pass_service.dart';
import 'closepass/incident_store.dart';
import 'display/display_controller.dart';
import 'link/ble_finder.dart';
import 'link/demo_main_module.dart';
import 'link/main_module.dart';
import 'link/transport.dart';
import 'navigation/nav_controller.dart';
import 'services/location.dart';
import 'settings.dart';
import 'ui/demo_photos.dart';
import 'ui/home_screen.dart';
import 'ui/theme.dart';

/// Everything the screens share.
class AppServices {
  final Settings settings;
  final MainModule module;
  final NavController nav;
  final DisplayController display;
  final IncidentStore store;
  final ClosePassService closePass;
  final ModuleFinder? finder;

  /// The built-in demo module while it is connected.
  DemoMainModule? demo;

  AppServices({
    required this.settings,
    required this.module,
    required this.nav,
    required this.display,
    required this.store,
    required this.closePass,
    this.finder,
  });

  /// The real app: Bluetooth, GPS, OpenRouteService, files in app storage.
  static Future<AppServices> create() async {
    final settings = await Settings.load();
    final finder = BleFinder();
    final module = MainModule(finder: finder, onRemember: settings.rememberDevice);
    final nav = NavController(
      routeProvider: () {
        if (!settings.hasOrsKey) {
          return (a, b) => Future.error(Exception('No OpenRouteService key. Add it in Settings.'));
        }
        return OrsClient(apiKey: settings.orsKey).route;
      },
      gps: gpsFixes,
    );
    final display = DisplayController(module: module, nav: nav);
    final docs = await getApplicationDocumentsDirectory();
    final store = IncidentStore(Directory('${docs.path}/incidents'));
    await store.load();
    final closePass = ClosePassService(
      module: module,
      store: store,
      settings: settings,
      display: display,
      locate: () async {
        final r = nav.rider;
        if (nav.active && r != null) return Where(r.lat, r.lng, 10);
        return quickWhere();
      },
    );
    final services = AppServices(
      settings: settings,
      module: module,
      nav: nav,
      display: display,
      store: store,
      closePass: closePass,
      finder: finder,
    );
    // Reconnect to the remembered Main Module, and finish any waiting work.
    final id = settings.deviceId;
    if (id != null) unawaited(module.connectToId(id, name: settings.deviceName));
    unawaited(closePass.resumePending());
    return services;
  }

  /// Uses the built-in demo module instead of real hardware.
  Future<DemoMainModule> connectDemo({DemoPhotoMaker? photos}) async {
    await module.disconnect();
    final d = DemoMainModule(photoMaker: photos ?? drawDemoPhotos);
    demo = d;
    d.done.then((_) {
      if (demo == d) demo = null;
    });
    await module.attach(d);
    return d;
  }

  Future<void> disconnect() async {
    demo = null;
    await module.disconnect();
  }
}

/// Makes [AppServices] available to every screen.
class AppScope extends InheritedWidget {
  final AppServices services;
  const AppScope({super.key, required this.services, required super.child});

  static AppServices of(BuildContext context) =>
      context.dependOnInheritedWidgetOfExactType<AppScope>()!.services;

  @override
  bool updateShouldNotify(AppScope old) => old.services != services;
}

class BikeNavApp extends StatelessWidget {
  final AppServices services;
  const BikeNavApp({super.key, required this.services});

  @override
  Widget build(BuildContext context) {
    return AppScope(
      services: services,
      child: MaterialApp(
        title: 'Bike Nav',
        debugShowCheckedModeBanner: false,
        theme: buildTheme(),
        home: const HomeScreen(),
      ),
    );
  }
}
