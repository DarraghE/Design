import 'dart:convert';
import 'dart:typed_data';

import 'package:http/http.dart' as http;

import 'incident.dart';

/// One photo for the plate reader, in time order.
class PlatePhoto {
  final Uint8List bytes;
  final String mimeType;
  final int offsetMs; // after the close pass was detected
  const PlatePhoto(this.bytes, {this.mimeType = 'image/jpeg', this.offsetMs = 0});
}

class PlateReaderException implements Exception {
  final String message;
  const PlateReaderException(this.message);
  @override
  String toString() => message;
}

/// Reads the overtaking vehicle's registration plate from the close-pass
/// photos with the OpenAI Chat Completions API (vision + structured output).
class PlateReader {
  final String apiKey;
  final String model;
  final http.Client _http;
  final Uri endpoint;
  final Duration timeout;

  PlateReader({
    required this.apiKey,
    required this.model,
    http.Client? client,
    Uri? endpoint,
    this.timeout = const Duration(seconds: 60),
  })  : _http = client ?? http.Client(),
        endpoint = endpoint ?? Uri.parse('https://api.openai.com/v1/chat/completions');

  static const systemPrompt = '''
You are a careful vehicle-registration reader for a bicycle safety system in Ireland.

WHAT HAPPENED
- A LiDAR on the back of a bicycle detected a motor vehicle overtaking the cyclist too closely. In Ireland traffic drives on the LEFT, so overtaking vehicles pass on the cyclist's RIGHT.
- A camera on the bicycle then took several photos in quick succession. They are given to you in time order. The first photo is taken shortly after the vehicle drew level with the bicycle.

WHICH VEHICLE
- The overtaking vehicle is usually the largest and closest moving vehicle, on the right-hand side of the frame or crossing it, seen from behind or from the side, and getting smaller or moving further ahead from one photo to the next.
- It normally appears in several consecutive photos. Prefer a vehicle that does.
- Ignore parked vehicles, vehicles coming the other way, distant vehicles, vehicles in other lanes that are clearly not passing the bicycle, and anything on the bicycle or the rider.
- If several vehicles could be the one, choose the most likely one and lower the confidence.

READING THE PLATE
- Combine evidence from all photos: one photo may show characters that are blurred in another.
- Irish plates (white, black characters, often a blue "IRL" band on the left) have the form YY[S]-CC-N: a 2-digit year, then for 2013 and later a half-year digit 1 or 2, then a county code, then a 1-6 digit number. Examples: 241-D-12345, 08-KY-1234, 152-C-987. County codes: C, CE, CN, CW, D, DL, G, KE, KK, KY, L, LD, LH, LK, LM, LS, MH, MN, MO, OY, RN, SO, T, W, WH, WX, WW (older: TN, TS, WD). Write Irish plates with hyphens.
- UK plates (rear plate yellow) are like AB12 CDE; Northern Ireland plates are like ABZ 1234. Write them with one space as shown.
- Other plates: copy the characters exactly as printed, in capitals.
- Watch for look-alike characters (0/O/D, 1/I, 8/B, 5/S, 2/Z) and use the plate format to decide between them.
- NEVER invent characters. Write "?" for each character you cannot read. If you cannot read most of the plate, set plate_found to false and plate to "".

CONFIDENCE
- confidence is your probability (0 to 1) that the plate string is exactly right AND belongs to the overtaking vehicle. Use below 0.5 if any character is a guess or the vehicle choice is unclear.

Reply only with the JSON object required by the schema.''';

  static const Map<String, dynamic> schema = {
    'type': 'object',
    'additionalProperties': false,
    'required': [
      'plate_found',
      'plate',
      'plate_country',
      'confidence',
      'best_image',
      'vehicle_type',
      'vehicle_colour',
      'vehicle_make',
      'notes',
    ],
    'properties': {
      'plate_found': {'type': 'boolean', 'description': 'True only if most of the overtaking vehicle\'s plate is readable.'},
      'plate': {'type': 'string', 'description': 'Normalised plate, capitals, "?" for unreadable characters, "" if not found.'},
      'plate_country': {
        'type': 'string',
        'enum': ['IE', 'UK', 'OTHER', 'UNKNOWN'],
      },
      'confidence': {'type': 'number', 'description': '0 to 1.'},
      'best_image': {'type': 'integer', 'description': 'Number (1 = first) of the photo where the plate is clearest; 0 if none.'},
      'vehicle_type': {'type': 'string', 'description': 'car, van, SUV, lorry, bus, motorbike, or unknown.'},
      'vehicle_colour': {'type': 'string'},
      'vehicle_make': {'type': 'string', 'description': 'Make or model if recognisable, else unknown.'},
      'notes': {'type': 'string', 'description': 'One short sentence: which vehicle you chose and why.'},
    },
  };

  /// The request body (kept separate so tests can check it).
  Map<String, dynamic> buildRequest(List<PlatePhoto> photos, {double? distanceM, double? speedKmh, DateTime? time}) {
    final facts = <String>[
      if (time != null) 'Time: ${time.toIso8601String()}.',
      if (distanceM != null) 'Closest passing distance measured by the LiDAR: ${distanceM.toStringAsFixed(2)} m.',
      if (speedKmh != null) 'Estimated speed of the vehicle: ${speedKmh.round()} km/h.',
    ];
    return {
      'model': model,
      'messages': [
        {'role': 'system', 'content': systemPrompt},
        {
          'role': 'user',
          'content': [
            {
              'type': 'text',
              'text': 'Close pass detected. ${facts.join(' ')} '
                  '${photos.length} photos follow in time order: '
                  '${[for (var i = 0; i < photos.length; i++) 'photo ${i + 1} at +${(photos[i].offsetMs / 1000).toStringAsFixed(1)} s'].join(', ')}. '
                  'Identify the overtaking vehicle and read its registration plate.',
            },
            for (final p in photos)
              {
                'type': 'image_url',
                'image_url': {'url': 'data:${p.mimeType};base64,${base64Encode(p.bytes)}', 'detail': 'high'},
              },
          ],
        },
      ],
      'response_format': {
        'type': 'json_schema',
        'json_schema': {'name': 'plate_reading', 'strict': true, 'schema': schema},
      },
    };
  }

  Future<PlateResult> read(List<PlatePhoto> photos, {double? distanceM, double? speedKmh, DateTime? time}) async {
    if (apiKey.trim().isEmpty) throw const PlateReaderException('Add your OpenAI API key in Settings.');
    if (photos.isEmpty) throw const PlateReaderException('No photos to read.');
    final http.Response res;
    try {
      res = await _http
          .post(endpoint,
              headers: {'Authorization': 'Bearer ${apiKey.trim()}', 'Content-Type': 'application/json'},
              body: jsonEncode(buildRequest(photos, distanceM: distanceM, speedKmh: speedKmh, time: time)))
          .timeout(timeout);
    } catch (e) {
      throw PlateReaderException('Could not reach OpenAI (${e.runtimeType}). Check the internet connection.');
    }
    if (res.statusCode != 200) throw PlateReaderException(_httpError(res));
    return parseResponse(res.body, model: model);
  }

  /// Checks the key and model without sending photos. Null means OK,
  /// otherwise a message for the user.
  Future<String?> check() async {
    if (apiKey.trim().isEmpty) return 'Enter an OpenAI API key first.';
    try {
      final res = await _http
          .get(endpoint.replace(path: '/v1/models/$model'), headers: {'Authorization': 'Bearer ${apiKey.trim()}'})
          .timeout(const Duration(seconds: 20));
      return res.statusCode == 200 ? null : _httpError(res);
    } catch (e) {
      return 'Could not reach OpenAI (${e.runtimeType}).';
    }
  }

  static String _httpError(http.Response res) {
    String? apiMessage;
    try {
      apiMessage = ((jsonDecode(res.body) as Map)['error'] as Map?)?['message'] as String?;
    } catch (_) {}
    final detail = apiMessage == null ? '' : ' ($apiMessage)';
    return switch (res.statusCode) {
      401 => 'OpenAI rejected the API key. Check it in Settings.',
      403 => 'This API key can\'t use the model$detail.',
      404 => 'Model not available to this key. Change the model in Settings$detail.',
      429 => 'OpenAI rate limit or credits used up$detail.',
      >= 500 => 'OpenAI is having problems (${res.statusCode}). Try again later.',
      _ => 'OpenAI error ${res.statusCode}$detail',
    };
  }

  static PlateResult parseResponse(String body, {String model = ''}) {
    final Map<String, dynamic> message;
    try {
      message = ((jsonDecode(body) as Map)['choices'] as List).first['message'] as Map<String, dynamic>;
    } catch (_) {
      throw const PlateReaderException('Unexpected reply from OpenAI.');
    }
    if (message['refusal'] != null) throw PlateReaderException('OpenAI declined: ${message['refusal']}');
    final Map<String, dynamic> j;
    try {
      j = jsonDecode(message['content'] as String) as Map<String, dynamic>;
    } catch (_) {
      throw const PlateReaderException('OpenAI did not return the expected JSON.');
    }
    final found = j['plate_found'] as bool? ?? false;
    final plate = found ? normalisePlate(j['plate'] as String? ?? '') : '';
    final country = j['plate_country'] as String? ?? 'UNKNOWN';
    return PlateResult(
      found: found && plate.isNotEmpty,
      plate: plate,
      country: country,
      confidence: ((j['confidence'] as num?)?.toDouble() ?? 0).clamp(0, 1),
      bestImage: (j['best_image'] as num?)?.toInt() ?? 0,
      vehicleType: j['vehicle_type'] as String? ?? '',
      vehicleColour: j['vehicle_colour'] as String? ?? '',
      vehicleMake: j['vehicle_make'] as String? ?? '',
      notes: j['notes'] as String? ?? '',
      plausibleFormat: plausiblePlate(plate),
      model: model,
    );
  }
}

const _counties = 'C|CE|CN|CW|D|DL|G|KE|KK|KY|L|LD|LH|LK|LM|LS|MH|MN|MO|OY|RN|SO|T|TN|TS|W|WD|WH|WX|WW';
final _irish = RegExp('^\\d{2}[12]?-($_counties)-\\d{1,6}\$');
final _uk = RegExp(r'^[A-Z]{2}\d{2} [A-Z]{3}$');
final _ni = RegExp(r'^[A-Z]{2,3} \d{1,4}$');

/// Upper-case, tidy spacing and hyphens.
String normalisePlate(String s) {
  var p = s.toUpperCase().trim().replaceAll(RegExp(r'\s+'), ' ');
  p = p.replaceAll(RegExp(r'\s*-\s*'), '-');
  // "241 D 12345" -> "241-D-12345"
  final spacedIrish = RegExp('^(\\d{2}[12]?) ($_counties) (\\d{1,6})\$').firstMatch(p);
  if (spacedIrish != null) p = '${spacedIrish[1]}-${spacedIrish[2]}-${spacedIrish[3]}';
  return p;
}

/// Whether [plate] has the shape of an Irish, UK or Northern Irish plate.
bool plausiblePlate(String plate) => _irish.hasMatch(plate) || _uk.hasMatch(plate) || _ni.hasMatch(plate);
