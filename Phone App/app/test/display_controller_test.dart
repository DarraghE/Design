import 'package:bike_app/display/display_controller.dart';
import 'package:bike_app/link/demo_main_module.dart';
import 'package:bike_app/link/main_module.dart';
import 'package:bike_app/link/protocol.dart';
import 'package:bike_app/navigation/nav_controller.dart';
import 'package:bike_app/services/place_search.dart';
import 'package:bike_navigation/bike_navigation.dart';
import 'package:fake_async/fake_async.dart';
import 'package:flutter_test/flutter_test.dart';

import 'helpers.dart';

void main() {
  test('idle shows READY; a close pass shows CLOSE PASS, flashes, then goes back', () {
    fakeAsync((time) {
      final module = MainModule();
      final nav = NavController(routeProvider: () => (a, b) async => dublinRoute(), gps: () => const Stream.empty());
      final display = DisplayController(module: module, nav: nav);
      final demo = DemoMainModule(notifyGap: Duration.zero, photoDelay: Duration.zero);
      module.attach(demo);
      time.flushMicrotasks();
      expect(display.mode, DisplayMode.idle);
      expect(sameFrame(demo.lcd.value!.frame, renderIdle()), isTrue);

      display.showCloseAlert(const ClosePassAlert(1, minDistanceM: 0.8, speedKmh: 42));
      time.flushMicrotasks();
      expect(display.mode, DisplayMode.closePass);
      expect(sameFrame(demo.lcd.value!.frame, renderCloseAlert(distanceM: 0.8, speedKmh: 42)), isTrue);
      expect(demo.lcd.value!.flash, isTrue);

      time.elapse(const Duration(seconds: 2));
      expect(demo.lcd.value!.flash, isFalse, reason: 'flashing stops after 1.5 s');
      expect(display.mode, DisplayMode.closePass);

      time.elapse(const Duration(seconds: 5));
      expect(display.mode, DisplayMode.idle);
      expect(sameFrame(demo.lcd.value!.frame, renderIdle()), isTrue);
      display.dispose();
    });
  });

  test('the current frame is re-sent regularly, but unchanged frames are not spammed', () {
    fakeAsync((time) {
      final module = MainModule();
      final nav = NavController(routeProvider: () => (a, b) async => dublinRoute(), gps: () => const Stream.empty());
      final display = DisplayController(module: module, nav: nav, keepAlive: const Duration(seconds: 3));
      final demo = DemoMainModule(notifyGap: Duration.zero, photoDelay: Duration.zero);
      module.attach(demo);
      time.flushMicrotasks();
      final sent = module.framesSent;
      nav.clearDestination(); // notifies, but the screen doesn't change
      time.flushMicrotasks();
      expect(module.framesSent, sent);
      time.elapse(const Duration(seconds: 7));
      expect(module.framesSent, sent + 2);
      display.dispose();
    });
  });

  test('while navigating the handlebar shows only directions, even after a close pass', () async {
    final route = dublinRoute();
    final module = MainModule();
    final nav = NavController(
      routeProvider: () => (a, b) async => route,
      gps: () => const Stream.empty(),
      simulationInterval: const Duration(milliseconds: 1),
    );
    final display = DisplayController(module: module, nav: nav);
    final demo = DemoMainModule(notifyGap: Duration.zero, photoDelay: Duration.zero);
    await module.attach(demo);
    await nav.choose(Place("St Patrick's Cathedral", route.line.points.last), route.line.points.first);
    await nav.start(route.line.points.first, simulate: true);
    expect(display.mode, DisplayMode.navigation);
    await until(() => nav.state != null);
    display.showCloseAlert(const ClosePassAlert(3, minDistanceM: 0.7));
    expect(display.mode, DisplayMode.navigation, reason: 'navigation keeps the screen');

    // Every frame on the handlebar is exactly the navigation screen for a state.
    var checked = 0;
    void check() {
      final s = nav.state;
      final shown = demo.lcd.value;
      if (s == null || shown == null) return;
      if (sameFrame(shown.frame, renderOled(s))) checked++;
    }

    nav.addListener(check);
    await until(() => nav.arrived, timeout: const Duration(seconds: 20));
    await module.flush();
    nav.removeListener(check);
    expect(checked, greaterThan(10));
    expect(sameFrame(demo.lcd.value!.frame, renderOled(nav.state!)), isTrue);

    await nav.stop();
    await module.flush();
    expect(display.mode, DisplayMode.idle);
    expect(sameFrame(demo.lcd.value!.frame, renderIdle()), isTrue);
    display.dispose();
  });
}
